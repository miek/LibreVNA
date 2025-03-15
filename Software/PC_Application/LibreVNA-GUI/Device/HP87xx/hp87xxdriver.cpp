#include "hp87xxdriver.h"

#include "CustomWidgets/informationbox.h"
#include "Util/util.h"

#include <QTcpSocket>
#include <QDateTime>
#include <QApplication>

HP87xxDriver::HP87xxDriver()
    : DeviceTCPDriver("HP87xx")
{
    diffGen = new TraceDifferenceGenerator<VNAPoint>([=](const VNAPoint &p){
        VNAMeasurement m;
        m.Z0 = 50.0;
        m.pointNum = p.index;
        m.frequency = p.frequency;
        m.dBm = excitationPower;
        m.measurements = p.data;
        emit VNAmeasurementReceived(m);
    });

    traceReader.waitingForResponse = false;
}

HP87xxDriver::~HP87xxDriver()
{
    delete diffGen;
}

std::set<QString> HP87xxDriver::GetAvailableDevices()
{
    std::set<QString> ret;

    // attempt to establish a connection to check if the device is available and extract the serial number
    detectedDevices.clear();
    auto sock = QTcpSocket();
    for(auto address : getSearchAddresses()) {
        sock.connectToHost(address, 5025);
        if(sock.waitForConnected(50)) {
           // connection successful
            sock.waitForReadyRead(100);
            ret.insert(address.toString());
            sock.disconnect();
        }
    }
    return ret;
}

bool HP87xxDriver::connectTo(QString address_str)
{
    if(connected) {
        disconnect();
    }

    QHostAddress address(address_str);
    qDebug() << "Connecting to: " + address_str;
    dataSocket.connectToHost(address, 5025);

    // check if connection succeeds
    if(!dataSocket.waitForConnected(1000)) {
        // socket failed
        dataSocket.close();
        InformationBox::ShowError("Error", "TCP connection timed out");
        return false;
    }

    connect(&dataSocket, qOverload<QAbstractSocket::SocketError>(&QTcpSocket::errorOccurred), this, [this](QAbstractSocket::SocketError err) {
        if(err == QAbstractSocket::SocketTimeoutError) {
            // ignore, these are triggered by the query function
        } else {
            emit ConnectionLost();
        }
    }, Qt::QueuedConnection);

    // grab model information
    dataSocket.write("IDN?\n");
    dataSocket.write("++read\n");
    dataSocket.waitForReadyRead(100);
    auto line = QString(dataSocket.readLine());
    auto fields = line.split(",");
    if(fields.size() != 4) {
        dataSocket.close();
        InformationBox::ShowError("Error", "Invalid *IDN? response:" + line);
        return false;
    }

    this->serial = fields[2];

    info = Info();
    info.hardware_version = fields[1];
    info.firmware_version = fields[3].trimmed();

    const QStringList supportedDevices = {"8719D", "8720D", "8722D"};

    if(!supportedDevices.contains(info.hardware_version)) {
        dataSocket.close();
        InformationBox::ShowError("Error", "Invalid hardware version: " + info.hardware_version);
        return false;
    }

    info.supportedFeatures.insert(DeviceDriver::Feature::VNA);
    info.supportedFeatures.insert(DeviceDriver::Feature::VNAFrequencySweep);

    // Extract limits
    // TODO: not sure that this can be queried
    // (other than setting it out of range and reading back the clipped value)
    // could also do it based on `IDN` model number
    info.Limits.VNA.ports = 2;
    info.Limits.VNA.minFreq =    50000000;
    info.Limits.VNA.maxFreq = 20050000000;
    info.Limits.VNA.maxPoints = 1601;
    info.Limits.VNA.arbitaryPointsValues = false;
    info.Limits.VNA.validPointsValues = {3, 11, 21, 26, 51, 101, 201, 401, 801, 1601};
    info.Limits.VNA.minIFBW = 10;
    info.Limits.VNA.maxIFBW = 3700;
    info.Limits.VNA.mindBm = -70;
    info.Limits.VNA.maxdBm = 15;

    connected = true;

    // reset to default configuration
    dataSocket.write("RST\n");

    emit InfoUpdated();

    return true;
}

void HP87xxDriver::disconnect()
{
    traceReaderStop();
    connected = false;
    dataSocket.close();
}

DeviceDriver::Info HP87xxDriver::getInfo()
{
    return info;
}

std::set<DeviceDriver::Flag> HP87xxDriver::getFlags()
{
    return std::set<DeviceDriver::Flag>();
}

QString HP87xxDriver::getStatus()
{
    return "";
}

QStringList HP87xxDriver::availableVNAMeasurements()
{
    return {"S11", "S21", "S12", "S22"};
}

bool HP87xxDriver::setVNA(const VNASettings &s, std::function<void (bool)> cb)
{
    excitationPower = s.dBmStart;
    excitedPorts = s.excitedPorts;

    freqStart = s.freqStart;
    freqStop = s.freqStop;
    points = s.points;

    if(!traceReaderStop()) {
        emit ConnectionLost();
        return false;
    }

    // configure the sweep
    write("STAR" + QString::number(freqStart));
    write("STOP" + QString::number(freqStop));
    write("IFBW" + QString::number(s.IFBW));
    write("POIN" + QString::number(points));
    //write(":SOUR:POW "+QString::number(s.dBmStart));
    write("FORM5");
    write("TAKE4ON");

//    traceTimer.start(100);
    if(cb) {
        cb(true);
    }
    traceReaderRestart();
    return true;
}

bool HP87xxDriver::setIdle(std::function<void (bool)> cb)
{
    if(!connected) {
        return false;
    }
    if(!traceReaderStop()) {
        emit ConnectionLost();
        return false;
    }
//    traceTimer.stop();

    if(cb) {
        cb(true);
    }
    return true;
}

QStringList HP87xxDriver::availableExtRefInSettings()
{
    return {""};
}

QStringList HP87xxDriver::availableExtRefOutSettings()
{
    return {""};
}

bool HP87xxDriver::setExtRef(QString option_in, QString option_out)
{
    Q_UNUSED(option_in)
    Q_UNUSED(option_out)
    return false;
}

void HP87xxDriver::write(QString s)
{
    dataSocket.write(QString(s + "\n").toLocal8Bit());
    dataSocket.readAll();
}

void HP87xxDriver::handleIncomingData()
{
    traceReader.waitingForResponse = false;
    auto max_size = info.Limits.VNA.maxPoints * 2 * 4;
    char raw_data[max_size];
    std::vector<double> data;
    qDebug() << "Trying to read header";
    auto count = dataSocket.read(raw_data, 4);
    qDebug() << "Read " << count << " bytes";
    auto size = raw_data[2] + (raw_data[3] << 8);
    qDebug() << "Size: " << size;
    count = dataSocket.read(raw_data, size);
    qDebug() << "Read " << count << " bytes";
    for (int i = 0; i < count/4; i++) {
        auto sample = reinterpret_cast<float*>(&raw_data[4*i]);
        data.push_back(*sample);
    }

    if(traceReader.state == 0) {
        std::vector<double> xaxis;
        double freq_step = (freqStop - freqStart) / (points - 1);
        for (int i = 0; i < count/8; i++) {
            xaxis.push_back(freqStart + (i * freq_step));
        }
        traceReader.xaxis = xaxis;
    }

    QString name = availableVNAMeasurements()[traceReader.state];
    traceReader.data[name] = data;

    if(traceReader.state >= 3) {
        auto lastIndex = traceReader.xaxis.size();
        if(lastIndex > 0) {
            // Compile VNApoints
            std::vector<VNAPoint> trace;
            trace.resize(lastIndex);
            for(int i=0;i<lastIndex;i++) {
                trace[i].index = i;
                trace[i].frequency = traceReader.xaxis[i];
                std::map<QString, std::complex<double>> tracedata;
                for(auto d : traceReader.data) {
                    tracedata[d.first] = std::complex(d.second[i*2], d.second[i*2+1]);
                }
                trace[i].data = tracedata;
            }

            diffGen->newTrace(trace);
        }
        traceReader.state = 0;
    } else {
        // move on to next trace
        traceReader.state++;
    }
    traceReaderStatemachine();
}

bool HP87xxDriver::traceReaderStop(unsigned int timeout)
{
    traceReader.enabled = false;
    if(traceReader.waitingForResponse) {
        // already issued a command, needs to wait for the response parsing
        auto start = QDateTime::currentDateTimeUtc();
        while(traceReader.waitingForResponse) {
            if(start.msecsTo(QDateTime::currentDateTimeUtc()) >= timeout) {
                // timed out
                qWarning() << "Timed out waiting for trace reader to stop";
                return false;
            }
            QApplication::processEvents();
        }
        QObject::disconnect(&dataSocket, &QTcpSocket::readyRead, this, &HP87xxDriver::handleIncomingData);
        return true;
    } else {
        // already stopped
        QObject::disconnect(&dataSocket, &QTcpSocket::readyRead, this, &HP87xxDriver::handleIncomingData);
        return true;
    }
}

void HP87xxDriver::traceReaderRestart()
{
    traceReader.enabled = true;
    traceReader.data.clear();
    traceReader.xaxis.clear();
    traceReader.state = 0;
    dataSocket.readAll();
    traceReaderStatemachine();
}

void HP87xxDriver::traceReaderStatemachine()
{
    if(!traceReader.enabled) {
        return;
    }
    if(traceReader.state == 0) {
        write("SWPSTART");
    }

    write("OUTPPRE"+QString::number(traceReader.state+1));
    write("++read");
    traceReader.waitingForResponse = true;
    QObject::connect(&dataSocket, &QTcpSocket::readyRead, this, &HP87xxDriver::handleIncomingData, Qt::UniqueConnection);
    return;
}

#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <fftw3.h>
#include <complex>
#include <fstream>
#include <vector>
#include <cmath>
#include <boost/filesystem.hpp>
#include <thread>
#include <chrono>

void fft(std::vector<std::complex<float>> &iqRx, int nfft, std::vector<std::complex<float>> &fftBuffer)
{
    if (iqRx.size() < nfft)
    {
        throw std::runtime_error("Input buffer size is less than nfft");
    }

    fftBuffer.resize(nfft);
    fftw_complex *in, *out;
    fftw_plan p;
    in = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * nfft);
    out = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * nfft);

    for (int i = 0; i < nfft; i++)
    {
        in[i][0] = iqRx[i].real();
        in[i][1] = iqRx[i].imag();
    }

    p = fftw_plan_dft_1d(nfft, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
    if (!p)
    {
        fftw_free(in);
        fftw_free(out);
        throw std::runtime_error("FFTW plan creation failed");
    }

    fftw_execute(p);
    fftw_destroy_plan(p);
    fftw_free(in);

    for (int i = 0; i < nfft; i++)
    {
        fftBuffer[i] = std::complex<double>(out[i][0], out[i][1]);
    }
    fftw_free(out);
}

void transmit(uhd::usrp::multi_usrp::sptr tx_usrp, const std::vector<std::complex<float>> &iq)
{
    uhd::tx_metadata_t tx_md;
    tx_md.start_of_burst = true;
    tx_md.end_of_burst = false;
    tx_md.has_time_spec = false;
   const int repeatCount = 1e3;
    uhd::stream_args_t stream_args("fc32"); // complex floats
    for (int count = 0; count < repeatCount; ++count)
    {
        size_t num_tx_samps = tx_usrp->get_tx_stream(stream_args)->send(iq.data(), iq.size(), tx_md);
        // std::cout << "Number of samples transmitted: " << num_tx_samps << std::endl;
        tx_md.start_of_burst = false;
        tx_md.end_of_burst = false;
    }
    size_t num_tx_samps = tx_usrp->get_tx_stream(stream_args)->send(iq.data(), iq.size(), tx_md);

    tx_md.start_of_burst = false;
    tx_md.end_of_burst = true;
    tx_usrp->get_tx_stream(stream_args)->send("", 0, tx_md);
}

void receive(uhd::usrp::multi_usrp::sptr usrpRx, std::vector<std::complex<float>> &iqRx, int ueNum)
{

    std::string fileName;
    std::string location = "/home/migue/workarea/data/";
    fileName = location + "UE" + std::to_string(ueNum) + "_12032024.dat";
    int captureLength(76800 * 100);
    uhd::rx_metadata_t md;
    uhd::stream_args_t stream_args("fc32"); // complex floats
    uhd::rx_streamer::sptr rx_stream = usrpRx->get_rx_stream(stream_args);
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_MORE);
    stream_cmd.stream_now = true;
    stream_cmd.num_samps = captureLength;

    // Process received samples (e.g., save to file, perform FFT, etc.)

    std::ofstream outfile(fileName, std::ios::binary);
    if (!outfile.is_open())
    {
        std::cerr << "Failed to open file for writing." << std::endl;
        return;
    }
  

    iqRx.resize(captureLength);
    size_t numRxSamps;
    while (true)
    {
        numRxSamps = rx_stream->recv(iqRx.data(), iqRx.size(), md, 3.0);
        if (numRxSamps != captureLength)
        {
            std::cerr << boost::format("Number of samples to small running again %d") % numRxSamps << std::endl;
            rx_stream->issue_stream_cmd(stream_cmd);
            continue;
        }
        else
        {
            break;
        }
    }
    outfile.write(reinterpret_cast<const char *>(iqRx.data()), iqRx.size() * sizeof(std::complex<float>));
    outfile.close();
    std::cout << boost::format("number of samples received %d") % numRxSamps << std::endl;
    std::vector<std::complex<float>> fftBuffer;
   
   
}

void reportParameters(const double &freq, const double &rate, const double &gain, const double &bw,const float &power)
{
    std::ofstream outfile("/home/migue/workarea/data/parameters.txt");
    if (!outfile.is_open())
    {
        std::cerr << "Failed to open file for writing parameters." << std::endl;
        return;
    }

    
    outfile << boost::format("Actual RX Freq: %f MHz") % (freq / 1e6) << std::endl;
    outfile << boost::format("Actual TX Rate: %f Msps") % (rate / 1e6) << std::endl;
    outfile << boost::format("Power %f dB") % power << std::endl;

    outfile.close();
}

float getPower(std::vector<std::complex<float>> &iq)
{
    float power(0.0);
    std::vector<std::complex<float>> fftBuffer;
    fftBuffer.resize(4096);
    fft(iq, 4096, fftBuffer);
    for (int i = 0; i < fftBuffer.size(); i++)
    {
        power += std::norm(fftBuffer[i]);
    }
    power = 10 * log10(power / fftBuffer.size());
    return power;
}


int UHD_SAFE_MAIN(int argc, char *argv[])
{
    uhd::set_thread_priority_safe();
    std::string serial;
    int ueNum(0);

    switch (ueNum)
    { // B210
    case 0:
        serial = "serial=3113EE5";
        break;
    case 1:
        serial = "serial=30AFF60";
        break;
    case 2:
        serial = "serial=30AE337";
        break;
        // B205-mini
    case 3:
        serial = "serial=30D3ACC";
        break;
    case 4:
        serial = "serial=F2B8AA";
        break;
    case 5:
        serial = "serial=30AFF60";
    case 6:
        serial = "serial=F2B8AA";
        break;
    case 7:
        serial = "serial=F5AB91";
        break;

    default:
        std::cerr << "Invalid UE number" << std::endl;
        return EXIT_FAILURE;
    }

    std::string subdev;
    std::string txant("TX/RX");
    std::string rxant;
    std::string ref("internal");
        double gain(60);

    if (ueNum == 0 || ueNum == 3)
    {
        rxant = "RX0";
        subdev = "A:A";
        gain = 60;

    }
    else
    {
        rxant = "RX2";
        gain = 30;
        subdev = "A:0";

    }

    std::string device_args((std::string)serial);

    double freq(4000e6);
    double bw(10e6);
    double rate(12.288e6);

    std::vector<std::complex<float>> iq;
    std::vector<std::complex<float>> fftBuffer;

    std::cout << boost::format("Creating the usrp device with: %s...") % device_args << std::endl;

    // Test IQ file first
    std::streampos size;
    char *memblock;
    /* Set TX device*/
    // Creating multi USRP device
    uhd::usrp::multi_usrp::sptr usrpTx = uhd::usrp::multi_usrp::make(device_args);
    // set clock source
    usrpTx->set_clock_source(ref);
    // set subdevice spec
    // usrp->set_rx_subdev_spec(subdev);
    usrpTx->set_tx_subdev_spec(subdev);
    // get printable summary of the device
    std::cout << boost::format("Using device:  %s") % usrpTx->get_pp_string() << std::endl;
    usrpTx->set_tx_rate(rate);
    // set the center frequency
    uhd::tune_request_t tune_request(freq);
 
    usrpTx->set_tx_antenna(txant);
    usrpTx->set_tx_bandwidth(bw);
    usrpTx->set_tx_gain(gain);
    usrpTx->set_tx_freq(tune_request);



    /* Set The RX device */
    std::string subdevRx = "A:0";
    std::string rxDeviceArgs = "addr=192.168.12.2";
    uhd::usrp::multi_usrp::sptr usrpRx = uhd::usrp::multi_usrp::make(rxDeviceArgs);
    usrpRx->set_clock_source(ref);
    usrpRx->set_rx_subdev_spec(subdevRx);
    usrpRx->set_rx_antenna("RX1");
    usrpRx->set_rx_rate(rate); 
    usrpRx->set_rx_freq(freq);
    usrpRx->set_rx_gain(40);
    std::string txFileName = "puschandsrsWave.dat";
    std::ifstream file(txFileName, std::ios::in | std::ios::binary | std::ios::ate);
    if (!file.is_open())
    {

        std::cerr << boost::format("Failed to open for reading pusch file") << std::endl;
        return EXIT_FAILURE;
    }

    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst = false;
    md.has_time_spec = false;
    size = file.tellg();
    memblock = new char[size];
    file.seekg(0, std::ios::beg);
    file.read(memblock, size);
    file.close();
    std::vector<float> interleavedIQ;
    float *doubleData = reinterpret_cast<float *>(memblock);
    int sampleSize = size / sizeof(std::complex<float>);

    std::cout << boost::format("Size of samples: %d") % sampleSize << std::endl;
    double timeOccupied = (1 / rate) * sampleSize;
    int numberOfPackets = 100;
    int totalSamples = sampleSize * numberOfPackets;

    double totalTime = timeOccupied * numberOfPackets;

    double peak(0.0);
    iq.resize(sampleSize);
    for (int i = 0; i < sampleSize; i++)
    {
        iq[i] = std::complex<double>(doubleData[i * 2], doubleData[(i * 2) + 1]);
    }  

    delete[] memblock;


    std::vector<std::complex<float>> iqRx;
    iqRx.resize(76800 * 100);
    std::thread txthread(transmit, usrpTx, iq);
    std::thread rxthread(receive, usrpRx, std::ref(iqRx), std::ref(ueNum));
    txthread.join();
    rxthread.join();
    float power = getPower(iqRx);
    std::cout << boost::format("Power of received signal: %f") % power << std::endl;
   
    reportParameters(freq, rate, gain, bw,power);

 
    return EXIT_SUCCESS;
}

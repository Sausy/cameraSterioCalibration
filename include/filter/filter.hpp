#ifndef _FILTER_HPP_
#define _FILTER_HPP_

class filter{
  public:
      //constructur
      //max Data defines the amounter of data stored in data Buffer
      filter(uint16_t maxData);

      //calculates the Mean and the Standard Deviation and
      //pushes the values to mean and variance
      bool stdDeviationFilter(double inData);
      double mean;
      double variance;

      //calculates the mean value of N data points
      double calcMean(const double * data_, uint8_t N);
      double calcMean(const std::vector<double> data_);

      //calculates the standardDeviation return (sqrt(abweichung^2))
      //returns it as a double
      double standardDeviation(const double * data_, double mean_, uint8_t N);
      double standardDeviation(const std::vector<double> data_, double mean_);



    private:
      std::vector<double> dataBuffer;
      uint16_t dataCounter;
      uint16_t N;
};


#endif

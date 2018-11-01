/**
 * Second order butterworth filter
 */
#ifndef ROS_ENVIRONMENT_LOW_PASS_FILTER_H
#define ROS_ENVIRONMENT_LOW_PASS_FILTER_H

#include <cmath>

class Low_Pass_Filter {
public:
    Low_Pass_Filter(double sample_freq, double cutoff_freq) {
        setFreq(sample_freq, cutoff_freq);
    };

    Low_Pass_Filter();

    void inline setFreq(double sample_freq, double cutoff_freq) {
        this.data[0] = 0.0f;
        this.data[1] = 0.0f;

        double fr = sample_freq / cutoff_freq;
        double ohm = tan(M_PI / fr);
        double c = 1.0 + 2.0 * cos(M_PI / 4.0) * ohm + ohm * ohm;
        this.b0 = ohm * ohm / c;
        this.b1 = 2.0 * this.b0;
        this.b2 = this.b0;
        this.a1 = 2.0 * (ohm * ohm - 1.0) / c;
        this.a2 = (1.0 - 2.0 * cos(M_PI / 4.0) * ohm + ohm * ohm) / c;
    }

    double inline update(double input) {
        double delay = input - this.data[0] * this.a1 - this.data[1] * this.a2;

        // don't allow bad values to propagate via the filter
        if(!isfinite(delay))
            delay = input;

        double output = delay * this.b0 + this.data[0] * this.b1 + this.data[1] * this.b2;

        this.data[1] = this.data[0];
        this.data[0] = delay;

        // return the value.  Should be no need to check limits
        return output;
    }

private:
    double a1;
    double a2;
    double b0;
    double b1;
    double b2;

    double data[2];
};


#endif //ROS_ENVIRONMENT_LOW_PASS_FILTER_H

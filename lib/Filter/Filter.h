#ifndef FILTER_H
#define FILTER_H

class Filter {
    public:
        Filter(float cutoff_freq, float sample_rate) {
            beta_ = exp(-cutoff_freq / sample_rate);
        }
        
        float filter(float signal) {
            float next_output = beta_ * prev_output_ + (1 - beta_) * signal;
            prev_output_ = next_output;

            return next_output;
        }

        void reset() {
            prev_output_ = 0;
        }
    
    private:
        float beta_;
        float prev_output_;
};

#endif
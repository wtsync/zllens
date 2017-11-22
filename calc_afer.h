#ifndef CALC_AFER_H
#define CALC_AFER_H

class Calc_afer {
public:
    Calc_afer(unsigned int min_, unsigned int max_);

    bool reset(unsigned int f, unsigned char range, unsigned char gap, unsigned int* first_f);
    // return false when full
    bool set_lap_get_next_f(unsigned int lap, unsigned int * f);
    bool get_best_f(unsigned int* f, unsigned int* lap);

private:
    unsigned int* p_lap;
    unsigned int* p_f;

    unsigned int count;
    unsigned int indexer;
    unsigned int last_pos;
    unsigned int min_tar;
    unsigned int max_tar;
};

#endif

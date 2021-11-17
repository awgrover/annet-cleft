#pragma once
#include "array_size.h"
#include <histo.h>

class CollectStats {
  public:
    // rescale to unsigned int
    static constexpr float IRMin = 17.0; // 10c = 50 F
    static constexpr float IRMax = 38.0; // 43=110 F. camera is actual 0..80C, in 0.25 resolution
    // max 64 bins after all
    static constexpr int Bins = (IRMax - IRMin) * 4 ; // .25 increments

    float min_v, max_v;
    Histogram<float> *histo;
    int first_high_temp_i; // calc'd and cached by find_histo_gap_and_cache

    CollectStats() {
      histo = new Histogram<float>(Bins, IRMin, IRMax);
      reset();
    }

    void reset() {
      min_v = 65535;
      max_v = 0;
      first_high_temp_i = 0;

      histo->reset();
    }

    void value(float newv) {
      if (newv < IRMin) newv = IRMin;
      if (newv > IRMax) newv = IRMax;

      if (min_v > newv) min_v = newv;
      if (max_v < newv) max_v = newv;

      histo->value(newv);

    }

    void print_histo() {
      Serial << F("histo[") << Bins << F(",");
      for (int i = 0; i < histo->bucket_ct; i++) {
        Serial << ( (int) histo->buckets[i] ) << F(",");
      }
      Serial << F("]") << endl;
      float first_high_temp = find_histo_gap_and_cache();
      Serial << F("firsthigh[") << first_high_temp_i << F(",") << first_high_temp << F("]") << endl;
    }

    float find_histo_gap_and_cache() {
      // we assume that there is a bunch of low-temp (background) values
      // across a bell-shaped histo,
      // and then a gap,
      // and then the body-temp (few)
      // Find the max temp before the gap
      // return 0 if not found
      int last_low_temp_i = find_at_least_in_a_row_over_fence<true>(0, 4, 1); // starting, at_least_in_a_row, fence
      if (last_low_temp_i == 0) {
        first_high_temp_i = 0;
        return 0;
      }
      int last_gap_temp_i = find_at_least_in_a_row_over_fence<false>(last_low_temp_i, 2, 1);
      if (last_low_temp_i == 0) {
        first_high_temp_i = 0;
        return 0;
      }

      first_high_temp_i = last_gap_temp_i + 1;
      return histo->bucket_value(first_high_temp_i);
    }

  private :

    template <boolean greater>
    float find_at_least_in_a_row_over_fence(int starting_i, const int at_least_in_a_row, int fence) {
      // find at_least_in_a_row buckets in a row that are ct >= fence
      // (reversed means <= fence)

      Serial << F("find fenced at_least_in_a_row ") << at_least_in_a_row << (greater ? F(" > ") : F(" < ")) << fence << F(" starting [] ") << starting_i << endl;

      int fenced_i = starting_i;
      int fenced_ct = 0;

      while (fenced_ct < at_least_in_a_row) { // restart first group until ct found

        // find next temp w/counts above fence
        boolean found = false;
        for (; fenced_i < histo->bucket_ct - at_least_in_a_row; fenced_i++) {
          if (greater ? (histo->buckets[fenced_i] > fence) : (histo->buckets[fenced_i] < fence) ) {
            found = true;
            Serial << F("Min start ") << fenced_i << F("/") << histo->bucket_value(fenced_i) << endl;
            break;
          }
        }

        if (! found) {
          Serial << F("no 0 buckets found by ") << fenced_i << endl;
          return 0;
        }

        // find at_least_in_a_row below-fence in a row
        fenced_ct = 0;
        for (int i = fenced_i; i < histo->bucket_ct - at_least_in_a_row && i < fenced_i + at_least_in_a_row; i++) {
          if (greater ? (histo->buckets[i] <= fence) : (histo->buckets[i] >= fence)) {
            break;
          }
          fenced_ct += 1; // gauranteed fence_ct at least 1
        }

        if (fenced_ct < at_least_in_a_row) {
          Serial << F("Not at_least_in_a_row in a row ") << at_least_in_a_row << F(" at [] ") << (fenced_i + fenced_ct - 1) << endl;
          // so, will retry till we run out of bins
          fenced_i += fenced_ct; // go past for next
        }
      }
      // we have at_least_in_a_row in a row (in fact, fenced_ct in a row)

      Serial << F("Fenced start [] ") << fenced_i << F(" .. ") << (fenced_ct + fenced_i - 1) << endl;
      return (fenced_ct + fenced_i - 1); // last value
    }
};

#include "SampleFilter.h"

static int filter_taps[SAMPLEFILTER_TAP_NUM] = {
  -4,
  -1,
  0,
  1,
  4,
  9,
  15,
  22,
  31,
  40,
  50,
  60,
  69,
  77,
  83,
  87,
  88,
  87,
  83,
  77,
  69,
  60,
  50,
  40,
  31,
  22,
  15,
  9,
  4,
  1,
  0,
  -1,
  -4
};

void SampleFilter_init(SampleFilter* f) {
  int i;
  for(i = 0; i < SAMPLEFILTER_TAP_NUM; ++i)
    f->history[i] = 0;
  f->last_index = 0;
}

void SampleFilter_put(SampleFilter* f, int input) {
  f->history[f->last_index++] = input;
  if(f->last_index == SAMPLEFILTER_TAP_NUM)
    f->last_index = 0;
}

int SampleFilter_get(SampleFilter* f) {
  long long acc = 0;
  int index = f->last_index, i;
  for(i = 0; i < SAMPLEFILTER_TAP_NUM; ++i) {
    index = index != 0 ? index-1 : SAMPLEFILTER_TAP_NUM-1;
    acc += (long long)f->history[index] * filter_taps[i];
  };
  return acc >> 10;
}

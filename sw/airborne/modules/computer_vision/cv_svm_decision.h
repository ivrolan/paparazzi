/**
 * @file modules/computer_vision/cv_svm_decision.h
 * Filters the image and uses the downsampled pixels to make a decision with a SVM
 */

#ifndef SVM_CV_H
#define SVM_CV_H

#include <stdint.h>
#include <stdbool.h>

struct svm_decision_msg_t {
  // uint16_t count_left;
  // uint16_t count_center;
  // uint16_t count_right;
  float value; // val of the decision function
  bool decision;
  bool updated;
};


// Module settings
extern uint8_t cod_lum_min;
extern uint8_t cod_lum_max;
extern uint8_t cod_cb_min;
extern uint8_t cod_cb_max;
extern uint8_t cod_cr_min;
extern uint8_t cod_cr_max;

// takes a look to the "lower_pix" pixels
extern uint8_t lower_pix;

extern bool cod_draw;


// Module functions
extern void filter_ground_init(void);
extern void filter_ground_periodic(void);

#endif /* SVM_CV_H */

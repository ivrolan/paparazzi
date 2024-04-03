/**
 * @file "modules/svm_follower_guided/svm_follower_guided.h"
 * Follow the decisions of a SVM
 */

#ifndef SVM_FOLLOWER_H
#define SVM_FOLLOWER_H

// settings
extern float oa_color_count_frac;

// functions
extern void svm_follower_init(void);
extern void svm_follower_periodic(void);

#endif // SVM_FOLLOWER_H


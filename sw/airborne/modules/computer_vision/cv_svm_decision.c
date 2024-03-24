
#include "modules/computer_vision/cv_svm_decision.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"


#define PRINT(string,...) fprintf(stderr, "[cv_svm_decision->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)


#ifndef SVM_DECISION_FPS
#define SVM_DECISION_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif

static pthread_mutex_t mutex;

// filter settings
uint8_t cod_lum_min = 0;
uint8_t cod_lum_max = 0;
uint8_t cod_cb_min = 0;
uint8_t cod_cb_max = 0;
uint8_t cod_cr_min = 0;
uint8_t cod_cr_max = 0;


uint8_t lower_pix;

bool cod_draw = false;


struct svm_decision_msg_t svm_decision_msg;

void get_pix(uint8_t **buffer_ptr,const uint16_t x, const uint16_t y, const uint16_t w, const uint16_t h, uint8_t **yp,uint8_t **up, uint8_t **vp) {
    if (x % 2 == 0) {
        // Even x
        *up = buffer_ptr[y * 2 * w + 2 * x];      // U
        *yp = buffer_ptr[y * 2 * w + 2 * x + 1];  // Y1
        *vp = buffer_ptr[y * 2 * w + 2 * x + 2];  // V
        //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
      } else {
        // Uneven x
        *up = buffer_ptr[y * 2 * w + 2 * x - 2];  // U
        //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
        *vp = buffer_ptr[y * 2 * w + 2 * x];      // V
        *yp = buffer_ptr[y * 2 * w + 2 * x + 1];  // Y2
      }
}


// are we sure?
#define N_FEATURES 136

// size of this coeff array?
const float coeff[] = {
    -0.29309039136074766, 0.2855126472852318, -0.8815636193507481, 0.0, 0.0, 0.0, 0.0, -0.3337508185709339, 0.23844752986268913, -0.15382879127505972, 0.23929505300714823, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5682652762871319, 0.4647882095870011, 0.23929505300714823, 0.0, 0.0, 0.0719534099285286, 0.0, 0.0, 0.14656320814933838, 0.4933103353281445, 0.6486319256539266, 0.663852885783037, 0.8284074507977163, 0.0, 0.27159452992287464, 0.0, -0.3749108220788966, 0.2501885149777319, 0.8117795439714822, 0.9617818197030583, 0.0, 0.0, 0.14514857436763817, 0.0, 1.1866219733394845, 0.08744717527240337, 0.3925148762414158, -0.38173110991786613, -0.8629583206454123, -0.3645830538769453, 0.0, 0.0, 0.8081412213021018, -0.15069315253234916, -0.12304145668571342, 0.663852885783037, 0.4245578327758888, 0.0, 0.0, 0.0, 0.6317056704282755, 0.5234855735918738, -0.525328658931751, 0.3489268212557386, -0.3659140685961238, -0.4984317297048043, -1.1119499087596283, -0.07005461463543923, 1.2348396131201549, -0.11624982467874223, 0.12649444615486832, -0.14950490844906578, 0.2569200536867092, -0.1285377125795339, 0.0, -0.4984317297048043, 1.0152209601291105, -1.1058379293020348, -0.08002725588843024, -0.15319361002577997, 0.6313925646115038, -0.4984317297048043, -0.06636591305872509, 0.6977584776702289, 0.8618014018748432, 0.5803086456917608, -0.14819587452768074, 0.10963176824859033, -0.6433528736116896, -1.132226414156248, -0.003688701576714145, -0.9798100600651549, 1.2233444067196388, -0.35997686241684446, -0.3843231391033499, -0.7337095667579443, -0.43846720458161864, 0.1631476183175557, -0.4212387465552579, -0.003688701576714145, 0.8683895950398859, -0.5219829661155828, 0.07203485041539442, -0.07563101152015025, -0.386302374282904, -0.003688701576714145, 0.0, 0.0, 0.7865426274838512, 1.1332411382298015, -0.018672953765361433, -0.38129197758602357, 0.0, -0.07319516443910958, 0.0, -0.5148151610321557, 0.3544489765629956, 0.14556986472667627, 0.3091078448277635, 0.23929505300714823, 0.0, -0.38813735977827457, -0.3337508185709339, 0.0, -0.3915735602882702, 0.32059870830523396, -0.0846158505558704, 0.7376938040722347, 0.23792833072702183, -0.6336669774010782, -0.38813735977827457, 0.0, -0.3917182784415165, 0.2506162325930821, 0.07778854673411759, 0.0, 0.0, 0.0, 0.0, 0.0
};

const float intercept = -7.278287104923872;

static struct image_t *cam_callback(struct image_t *img);
static struct image_t *cam_callback(struct image_t *img __attribute__((unused))) {

  PRINT("img callback\n");

  // get image
  uint8_t *buffer = img->buf;
  uint8_t downsample_factor = 30;

  // create feature_vector
  uint8_t feature_vector[N_FEATURES];

  // downsample and filter
  uint16_t x_width = img->w / downsample_factor;
  uint16_t y_height = img->h / downsample_factor;

  // check that the downsampled image has area equal to the feat vector
  PRINT("Downsampled width from %d to %d pix\n",img->w, x_width);
  PRINT("Downsampled height from %d to %d pix\n",img->h, y_height);
  PRINT("Downsampled area from %d to %d | N_features: %d\n", img->w*img->h, x_width * y_height, N_FEATURES);

  uint16_t x_index = 0, y_index= 0;
  uint16_t feat_counter = 0;
  for (int16_t x = img->w; x >= downsample_factor; x-=downsample_factor) {
    x_index = (img->w - (uint16_t) x) / downsample_factor;
    PRINT("x_index is %d\n", x_index);
    for (uint16_t y = 0; y < img->h - downsample_factor; y+=downsample_factor) {  
      y_index = y / downsample_factor;
      PRINT("y_index is %d\n", y_index);
      //get_pix(&buffer, x, y,img->w, img->h, &yp, &up, &vp);
        uint8_t *yp, *up, *vp;
        // get color YUV
        if (x % 2 == 0) {
          // Even x
          up = &buffer[y * 2 * img->w + 2 * x];      // U
          yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
          vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V

        } else {

          // Uneven x
          up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
          vp = &buffer[y * 2 * img->w + 2 * x];      // V
          yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
        }

      // -- filter --
      PRINT("Access index to feat_vec is %d\n", y_index + x_index*y_height);
      if ( (*yp >= cod_lum_min) && (*yp <= cod_lum_max) &&
          (*up >= cod_cb_min ) && (*up <= cod_cb_max ) &&
          (*vp >= cod_cr_min ) && (*vp <= cod_cr_max )) {
        
        feature_vector[y_index + x_index*y_height] = 1;
      } else {
        feature_vector[y_index + x_index*y_height] = 0;
      }

      feat_counter++;
    }
  }
  
  PRINT("Feature vector got %d values", feat_counter);
  
  // multiply values by coefficients
  // as feature vector values are 1 or 0 maybe there is another way to make it more efficient
  // e.g. selecting only the coeffs we need
  float val = 0;
  for (int i = 0; i <  N_FEATURES; i++) {
    val += coeff[i] * feature_vector[i];
  }
  val += intercept;

  pthread_mutex_lock(&mutex);
  svm_decision_msg.value = val;
  svm_decision_msg.decision = svm_decision_msg.value > 0;
  svm_decision_msg.updated = true;
  // PRINT("Value is %.2f | Decision is %d\n", svm_decision_msg.value, svm_decision_msg.decision);
  pthread_mutex_unlock(&mutex);

  //PRINT("updated, lower pix is %d", lower_pix);
  return img;
}


extern void svm_decision_init(void) {
  svm_decision_msg.value = 0;
  svm_decision_msg.decision = false;
  svm_decision_msg.updated = false;

  pthread_mutex_init(&mutex, NULL);

  cod_lum_min = SVM_DECISION_LUM_MIN;
  cod_lum_max = SVM_DECISION_LUM_MAX;
  cod_cb_min = SVM_DECISION_CB_MIN;
  cod_cb_max = SVM_DECISION_CB_MAX;
  cod_cr_min = SVM_DECISION_CR_MIN;
  cod_cr_max = SVM_DECISION_CR_MAX;
  lower_pix = SVM_DECISION_LOWER_PIX;
  cod_draw = SVM_DECISION_DRAW;
  #define CALLBACK_ID 0
  // SVM_DECISION_CAM will be defined in the xml file
  cv_add_to_device(&SVM_DECISION_CAM, cam_callback, SVM_DECISION_FPS, 0);
  PRINT("INIT\n");
}

void svm_decision_periodic(void)
{ 
  // is it really necessary to create a new struct here?
  static struct svm_decision_msg_t  local_msg;
  // reading from ground_filter, that is the reason why 
  pthread_mutex_lock(&mutex);
  memcpy(&local_msg, &svm_decision_msg, sizeof(struct svm_decision_msg_t));
  pthread_mutex_unlock(&mutex);
  
  PRINT("periodic\n");

  if (local_msg.updated) {
    // it seems we have to send the plain values

    // if we have problems with the creation of a new msg, we can use the previous ground filter
    // we shouldn't use updated here, but let's leave like that to test compiling issues
    PRINT("Value is %.2f | Decision is %d\n", svm_decision_msg.value, svm_decision_msg.decision);

    // AbiSendMsgGROUND_FILTER_DETECTION(GROUND_FILTER_DETECTION_ID,
    //                             local_msg.value,
    //                             local_msg.decision,
    //                             local_msg.updated, 0);

    // PRINT("sending msg %d|%d|%d", local_msg.count_left,
    //                             local_msg.count_center,
    //                             local_msg.count_right);
    local_msg.updated = false;
  }
}

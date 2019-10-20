//Face detection
#define BB_RAW_WIDTH                        1280
#define BB_RAW_HEIGHT                       960
#define BB_FACE_MODEL                       "./data/face_vino"

//Camera field of view
#define BB_HFOV                                 62.2
#define BB_VFOV                                 48.8

//Tracking
#define TRACKING_MAX_AGE          0
#define TRACKING_MIN_HITS          5
#define TRACKING_MIN_IOU            0.05
#define TRACKING_NUM_COLORS  20

//pca9685 defines
#define BB_PCA_PIN_BASE 	300
#define BB_PCA_MAX_PWM 		4096 //max ticks in PWM signal
#define BB_PCA_HERTZ 		50

//calibrated: for SG90
#define BB_SERVO_MS_MIN 	0.6
#define BB_SERVO_MS_MAX 	2.4
#define BB_SERVO_MAX_SPEED	0.6 //degrees per ms - from datasheet

//from datasheet: for continuous rotation FS5103R
#define BB_CONT_MS_MIN		1.0
#define BB_CONT_MS_MAX		2.0

#define BB_HEAD_DT		10 //ms time span 
#define BB_HEAD_MAX_STEP	BB_HEAD_DT * BB_SERVO_MAX_SPEED
#define BB_HEAD_INIT_POS	50.0 //initial angle
#define BB_HEAD_MIN_LIMIT	40.0
#define BB_HEAD_MAX_LIMIT	90.0

//PINS
#define BB_PIN_EYEBROW 		BB_PCA_PIN_BASE  //eyebrow servo at pin location 0 on PCA controller
#define BB_PIN_HEAD		BB_PCA_PIN_BASE + 1 //head servo at pin location 1 on PCA controller
#define BB_PIN_ARM		BB_PCA_PIN_BASE + 2 //--
#define BB_PIN_LEFT_MOTOR	BB_PCA_PIN_BASE + 3
#define BB_PIN_RIGHT_MOTOR	BB_PCA_PIN_BASE + 4
#define BB_PIN_PCA		BB_PCA_PIN_BASE + 16 //all pins on PCA controller 
#define BB_PIN_SWITCH  0

//ADC
#define BB_IR_LEFT		0	//Infrared sensors
#define BB_IR_RIGHT		1

#define BB_IR_SCALER_LEFT       755.0
#define BB_IR_SCALER_RIGHT      634.5
#define BB_EWMA_GAMMA           0.7

//face search status
#define BB_NO_FACE 0
#define BB_FOUND_FACE 1
#define BB_FACE_FAR 2

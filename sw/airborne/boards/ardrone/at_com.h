
#include "math/pprz_algebra_float.h"

#ifndef BOARDS_ARDRONE_AT_COM_H
#define BOARDS_ARDRONE_AT_COM_H

//Define the AT_REF bits
typedef enum {
	REF_TAKEOFF		= 1U << 9,
	REF_EMERGENCY	= 1U << 8,
	REF_DEFAULT		= 0x11540000
} AT_REFS;

//Define control states
typedef enum {
  CTRL_DEFAULT,
  CTRL_INIT,
  CTRL_LANDED,
  CTRL_FLYING,
  CTRL_HOVERING,
  CTRL_TEST,
  CTRL_TRANS_TAKEOFF,
  CTRL_TRANS_GOTOFIX,
  CTRL_TRANS_LANDING,
  CTRL_TRANS_LOOPING,
  CTRL_NUM_STATES
} CTRL_STATES;

//Define the AR.Drone states
typedef enum {
  ARDRONE_FLY_MASK            = 1U << 0,  /*!< FLY MASK : (0) ardrone is landed, (1) ardrone is flying */
  ARDRONE_VIDEO_MASK          = 1U << 1,  /*!< VIDEO MASK : (0) video disable, (1) video enable */
  ARDRONE_VISION_MASK         = 1U << 2,  /*!< VISION MASK : (0) vision disable, (1) vision enable */
  ARDRONE_CONTROL_MASK        = 1U << 3,  /*!< CONTROL ALGO : (0) euler angles control, (1) angular speed control */
  ARDRONE_ALTITUDE_MASK       = 1U << 4,  /*!< ALTITUDE CONTROL ALGO : (0) altitude control inactive (1) altitude control active */
  ARDRONE_USER_FEEDBACK_START = 1U << 5,  /*!< USER feedback : Start button state */
  ARDRONE_COMMAND_MASK        = 1U << 6,  /*!< Control command ACK : (0) None, (1) one received */
  ARDRONE_CAMERA_MASK         = 1U << 7,  /*!< CAMERA MASK : (0) camera not ready, (1) Camera ready */
  ARDRONE_TRAVELLING_MASK     = 1U << 8,  /*!< Travelling mask : (0) disable, (1) enable */
  ARDRONE_USB_MASK            = 1U << 9,  /*!< USB key : (0) usb key not ready, (1) usb key ready */
  ARDRONE_NAVDATA_DEMO_MASK   = 1U << 10, /*!< Navdata demo : (0) All navdata, (1) only navdata demo */
  ARDRONE_NAVDATA_BOOTSTRAP   = 1U << 11, /*!< Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent */
  ARDRONE_MOTORS_MASK  	      = 1U << 12, /*!< Motors status : (0) Ok, (1) Motors problem */
  ARDRONE_COM_LOST_MASK       = 1U << 13, /*!< Communication Lost : (1) com problem, (0) Com is ok */
  ARDRONE_SOFTWARE_FAULT      = 1U << 14, /*!< Software fault detected - user should land as quick as possible (1) */
  ARDRONE_VBAT_LOW            = 1U << 15, /*!< VBat low : (1) too low, (0) Ok */
  ARDRONE_USER_EL             = 1U << 16, /*!< User Emergency Landing : (1) User EL is ON, (0) User EL is OFF*/
  ARDRONE_TIMER_ELAPSED       = 1U << 17, /*!< Timer elapsed : (1) elapsed, (0) not elapsed */
  ARDRONE_MAGNETO_NEEDS_CALIB = 1U << 18, /*!< Magnetometer calibration state : (0) Ok, no calibration needed, (1) not ok, calibration needed */
  ARDRONE_ANGLES_OUT_OF_RANGE = 1U << 19, /*!< Angles : (0) Ok, (1) out of range */
  ARDRONE_WIND_MASK 		  = 1U << 20, /*!< WIND MASK: (0) ok, (1) Too much wind */
  ARDRONE_ULTRASOUND_MASK     = 1U << 21, /*!< Ultrasonic sensor : (0) Ok, (1) deaf */
  ARDRONE_CUTOUT_MASK         = 1U << 22, /*!< Cutout system detection : (0) Not detected, (1) detected */
  ARDRONE_PIC_VERSION_MASK    = 1U << 23, /*!< PIC Version number OK : (0) a bad version number, (1) version number is OK */
  ARDRONE_ATCODEC_THREAD_ON   = 1U << 24, /*!< ATCodec thread ON : (0) thread OFF (1) thread ON */
  ARDRONE_NAVDATA_THREAD_ON   = 1U << 25, /*!< Navdata thread ON : (0) thread OFF (1) thread ON */
  ARDRONE_VIDEO_THREAD_ON     = 1U << 26, /*!< Video thread ON : (0) thread OFF (1) thread ON */
  ARDRONE_ACQ_THREAD_ON       = 1U << 27, /*!< Acquisition thread ON : (0) thread OFF (1) thread ON */
  ARDRONE_CTRL_WATCHDOG_MASK  = 1U << 28, /*!< CTRL watchdog : (1) delay in control execution (> 5ms), (0) control is well scheduled */
  ARDRONE_ADC_WATCHDOG_MASK   = 1U << 29, /*!< ADC Watchdog : (1) delay in uart2 dsr (> 5ms), (0) uart2 is good */
  ARDRONE_COM_WATCHDOG_MASK   = 1U << 30, /*!< Communication Watchdog : (1) com problem, (0) Com is ok */
  ARDRONE_EMERGENCY_MASK      = 1U << 31  /*!< Emergency landing : (0) no emergency, (1) emergency */
} ARDRONE_STATES;

//Navdata option packet without data
typedef struct _navdata_option_t {
	uint16_t  tag;
	uint16_t  size;
	uint8_t   data[1];
} __attribute__ ((packed)) navdata_option_t;

//Main navdata packet
typedef struct _navdata_t {
	uint32_t    header;			  /*!< Always set to NAVDATA_HEADER */
	uint32_t    ardrone_state;    /*!< Bit mask built from def_ardrone_state_mask_t */
	uint32_t    sequence;         /*!< Sequence number, incremented for each sent packet */
	uint32_t    vision_defined;

	navdata_option_t  options[1];
} __attribute__ ((packed)) navdata_t;

//Navdata checksum packet
typedef struct _navdata_cks_t {
	uint16_t  tag;
	uint16_t  size;
	uint32_t  cks;
} __attribute__ ((packed)) navdata_cks_t;

//Navdata demo option
typedef struct _navdata_demo_t {
	uint16_t			tag;					/*!< Navdata block ('option') identifier */
	uint16_t			size;					/*!< set this to the size of this structure */
	uint32_t			ctrl_state;             /*!< Flying state (landed, flying, hovering, etc.) defined in CTRL_STATES enum. */
	uint32_t			vbat_flying_percentage; /*!< battery voltage filtered (mV) */
	float				theta;                  /*!< UAV's pitch in milli-degrees */
	float				phi;                    /*!< UAV's roll  in milli-degrees */
	float				psi;                    /*!< UAV's yaw   in milli-degrees */
	int32_t				altitude;               /*!< UAV's altitude in centimeters */
	float				vx;                     /*!< UAV's estimated linear velocity */
	float				vy;                     /*!< UAV's estimated linear velocity */
	float				vz;                     /*!< UAV's estimated linear velocity */
	uint32_t			num_frames;			    /*!< streamed frame index */ // Not used -> To integrate in video stage.
	// Camera parameters compute by detection
	struct FloatMat33	detection_camera_rot;   /*!<  Deprecated ! Don't use ! */
	struct FloatVect3	detection_camera_trans; /*!<  Deprecated ! Don't use ! */
	uint32_t			detection_tag_index;    /*!<  Deprecated ! Don't use ! */
	uint32_t	  		detection_camera_type;  /*!<  Type of tag searched in detection */
	// Camera parameters compute by drone
	struct FloatMat33	drone_camera_rot;		/*!<  Deprecated ! Don't use ! */
	struct FloatVect3	drone_camera_trans;	    /*!<  Deprecated ! Don't use ! */
} __attribute__ ((packed)) navdata_demo_t;

//Navdata physical measures option
typedef struct _navdata_phys_measures_t {
  uint16_t				tag;
  uint16_t				size;

  float					accs_temp;
  uint16_t				gyro_temp;
  struct FloatVect3		phys_accs;
  struct FloatVect3		phys_gyros;
  uint32_t				alim3V3;              // 3.3volt alim [LSB]
  uint32_t				vrefEpson;            // ref volt Epson gyro [LSB]
  uint32_t				vrefIDG;              // ref volt IDG gyro [LSB]
} __attribute__ ((packed)) navdata_phys_measures_t;

//External functions
extern void init_at_com(void);
extern void at_com_recieve_navdata(unsigned char* buffer);
extern void at_com_send_config(char* key, char* value);
extern void at_com_send_ftrim(void);
extern void at_com_send_ref(int bits);
extern void at_com_send_pcmd(int mode, float thrust, float roll, float pitch, float yaw);
extern void at_com_send_calib(int device);

#endif /* BOARDS_ARDRONE_AT_COM_H */

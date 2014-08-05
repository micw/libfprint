#ifndef __ESS801U_H_
#define __ESS801U_H_

//Answer buffer header
#define ET_ANS_HDR 0x55,0x53,0x42,0x53
//Command buffer header
#define ET_CMD_HDR 0x55,0x53,0x42,0x43 
//Buffer permanent data (immediately after header on both command and answer)
#define ET_BUF_PDATA 0x08,0x90,0xf1,0x81 

//Commands
#define ECMD_GETIDENT 0x120C //Get device identification???
#define ECMD_READ_SHORT 0xD20C //Request a short buffer (512 bytes length) with unknown data
#define ECMD_WRITE_SHORT 0xDA0C //Writes back a short buffer received by ECMD_READ_SHORT (maybe changed first byte, but not always)
#define ECMD_GETNSTRING 0xD30C //Get device name string???
#define ECMD_SETREG 0xCF0A //Set register??? Purpose totally unknown, return buffer length always 13 (command result)
#define ECMD_READ_LONG 0xCF06 
/*Request a long buffer (1536 bytes length) with unknown data or pre-enroll buffer (6144 bytes length)
  or enroll buffer (61440 bytes length)*/
#define ECMD_READ_LONG 0xCF06

/*
Structure of SS801U command: (As I see it, maybe wrong)
1.  Header         4 bytes (see above)
2.  Permanent data      4 bytes (see above)
3.  Flag1      2 bytes (Need to initialize)
4.  Empty data     4 bytes
5.  Command        2 bytes (Need to initialize)
6.  Flag2      2 bytes (Need to initialize)
7.  Flag3      2 bytes (Need to initialize)
8.  Flag4      2 bytes (Need to initialize)
9.  Flag5      2 bytes (Need to initialize)
10. Flag6      2 bytes (Need to initialize)
11. Empty data     5 bytes
TOTAL: 31 bytes
*/

struct et_cmd
{
 unsigned short flag1; //Two bytes after header and permanent data
 unsigned short cmd; //I think, this is command.
 unsigned short flag2; //
 unsigned short flag3; //
 unsigned short flag4; //I don't know what is it...
 unsigned short flag5; //
 unsigned short flag6; //
};


struct et_init
{
 int ssm_state;
 int stage; //Command stage (0 to 59)
 int rstage; //Flag for buffer received and confirmed by 13 bytes answer buffer
};


struct init_data
{
 struct et_init* init;
 struct fpi_ssm* ssm;
};

struct poll_data
{
 struct fpi_img_dev* dev;
 struct et_init* init;
};

/*Number of stages of initialization process before we can begin 
waiting for finger*/
#define ET_ACTIVATE_STATES 118
#define ET_CAPTURE_STATES 82

//FOR DEBUG, MUST BE REMOVED!
char* print_cmd_buf();
char* print_buf(char* buf);

static void start_capture(struct fp_img_dev *dev);
static void complete_deactivation(struct fp_img_dev *dev);

char* et_get_row(char* ibuf,int num);
void build_init_cmd(struct et_cmd* cmd,int stage);
void fill_init_cmd(struct et_cmd* cmd);
void build_enroll_cmd(struct et_cmd* cmd,int stage);
void build_poll_cmd(struct et_cmd* cmd);
void free_img_buf();

static void et_write_init_cb(struct libusb_transfer* transfer);
static void et_read_answer_cb(struct libusb_transfer* transfer);
static void et_write_poll_cb(struct libusb_transfer* transfer);
static void et_read_poll_cb(struct libusb_transfer* transfer);
static void et_write_enroll_cb(struct libusb_transfer* transfer);
static void et_read_enroll_cb(struct libusb_transfer* transfer);

/*
Parameters:
dev - device
init - stage information
user_data - init_data
*/
int et_write_init(struct fp_img_dev* dev,struct et_init* dinit,void* user_data);
/*
Parameters:
dev - device
init - stage information
user_data - init_data
*/
int et_read_answer(struct fp_img_dev* dev,struct et_init* dinit,void* user_data);
int et_write_poll(struct fp_img_dev* dev,struct poll_data* adata);
int et_read_poll(struct fp_img_dev* dev,struct poll_data* adata);
/*
Parameters:
dev - device
init - stage information
user_data - init_data
*/
int et_write_enroll_data(struct fp_img_dev* dev,struct et_init* dinit,void* user_data);
int et_read_enroll_data(struct fp_img_dev* dev,struct et_init* dinit,void* user_data);
/*Verifies answer buffer for validity (enroll)*/
int et_verify_enroll_data(struct et_init* dinit,int len);
/*Verifies answer buffer for validity (device init)*/
int et_verify_answer(struct et_init* dinit,int len);
/*Verifies transfer result*/
int et_verify_result(char* result);

static void start_finger_detection(struct fp_img_dev *dev,struct poll_data* adata);

#endif /*__ESS801U_H_*/

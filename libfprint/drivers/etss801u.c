/*
 * EgisTec SS801U driver for libfprint
 * Copyright (C) 2011 Alexey Prokopchuk <alexpro@homelan.lg.ua>
 *
 * Initially based on code from libfprint aes1610 driver.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#define FP_COMPONENT "etss801u"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <libusb.h>

#include <fp_internal.h>
#include "driver_ids.h"
#include "etss801u.h"

#define EP_IN          (2 | LIBUSB_ENDPOINT_IN)
#define EP_OUT         (1 | LIBUSB_ENDPOINT_OUT)

#define BULK_TIMEOUT 4000

 
/* The SS801U is an imaging device using a swipe-type sensor.  
 * It sending eight 768 x 80 frames for each fingerprint scan session. 
 * All those frames are simply collected into the whole image.
 * But, 768 x 80 image is too fuzzy and big, and causing segfaults
 * after scan. So when I build result image, only every fourth byte
 * of every fourth row saved. 
 * Hence, image size always 192 x 160.
 */ 


/*Command and answer static buffers*/
char cmd_buf[]={ET_CMD_HDR,ET_BUF_PDATA,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
char ret_buf[8192];
/*SS801U command structure*/
static struct et_init einit;
/*Dynamic image fragmets array*/
char* et_img_buf[]={NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};


struct etss801u_dev {
   gboolean deactivating;
};

/*Poll functions (when we awaiting for finger)*/
void build_poll_cmd(struct et_cmd* cmd)
{
 memset(cmd,0,sizeof(*cmd));
 cmd->flag1=0x200;
 cmd->cmd=ECMD_READ_SHORT;
 cmd->flag2=0x1A00;
 cmd->flag4=0xB0;
 cmd->flag5=0x100;
 fill_init_cmd(cmd);
}

int et_read_poll(struct fp_img_dev* dev,struct poll_data* adata)
{
 int r;

 struct etss801u_dev* edev=dev->priv;
 if(edev->deactivating)
 {
  complete_deactivation(dev);
  return 0;
 }
 struct libusb_transfer* transfer=libusb_alloc_transfer(0);
 if(!transfer) return -ENOMEM;
 libusb_fill_bulk_transfer(transfer,dev->udev,EP_IN,(char*)&ret_buf,8192,et_read_poll_cb,adata,BULK_TIMEOUT);
 r=libusb_submit_transfer(transfer);
 if(r<0)
 {
  r=-EIO;
  libusb_free_transfer(transfer);
 }
 return r;
}

static void et_write_poll_cb(struct libusb_transfer* transfer)
{
 int r;
 
 struct poll_data* adata=transfer->user_data;
 struct fpi_img_dev* dev=adata->dev;
 if(transfer->status!=LIBUSB_TRANSFER_COMPLETED)
 {
  libusb_free_transfer(transfer);
  fpi_imgdev_session_error(dev,-EIO);
  return;
 }
 if(transfer->actual_length!=31)
 {
  libusb_free_transfer(transfer);
  fpi_imgdev_session_error(dev,-EIO);
  return;
 }
 libusb_free_transfer(transfer);
 r=et_read_poll(dev,adata);
 if(r)
 {
  libusb_free_transfer(transfer);
  fpi_imgdev_session_error(dev,r);
  return;
 }
}

static void et_read_poll_cb(struct libusb_transfer* transfer)
{
 int len;
 char* br;
 int r;
 
 struct poll_data* adata=transfer->user_data;
 struct fpi_img_dev* dev=adata->dev;
 if(transfer->status!=LIBUSB_TRANSFER_COMPLETED)
 {
  /*WORKAROUND NEEDED, SOMETIME ALL FREEZES, RESET DEVICE AND REACTIVATE???*/
  libusb_free_transfer(transfer);
  fp_dbg("ERROR");
  fpi_imgdev_session_error(dev,-EIO);
  //return;
  goto _st;
 }
 len=transfer->actual_length-13;
 if(len!=512)
 {
  fp_dbg("Invalid return buffer, length: %d",transfer->actual_length);
  libusb_free_transfer(transfer);
  fpi_imgdev_session_error(dev,-EPROTO);
  return;
 }
 libusb_free_transfer(transfer);
 //Check poll cmd result
 br=(char*)&ret_buf[len];
 r=et_verify_result(br);
 if(r)
 {
  fp_dbg("Invalid result: %s",print_buf(br));
  fpi_imgdev_session_error(dev,-EPROTO);
  return;
 }
 //fp_dbg("0x%02hhX,0x%02hhX",ret_buf[0],ret_buf[1]);
 if((unsigned char)ret_buf[0]==0x82)
 {
  if(adata->init->rstage<20) 
  {
  /*VERY UGLY workaround below, but I don't know how to deal with it.
  Sometime ret_buf[0] equals 0x82 at the first poll rounds, but
  really no finger on the scanner
  */ 
   fp_dbg("POLL WORKAROUND");
   goto _st;
  }
  //Finger detected
  //fp_dbg("FINGER DETECTED!");
  //fp_dbg("BUF: %s",print_buf(ret_buf));
  fpi_imgdev_report_finger_status(dev,TRUE);
  start_capture(dev);
  return;
 }
 //No finger detected
_st:
 adata->init->rstage++; 
 start_finger_detection(dev,adata);
}

int et_write_poll(struct fp_img_dev* dev,struct poll_data* adata)
{
 struct et_cmd ecmd;
 int r;
 struct timeval tv;
 
 build_poll_cmd(&ecmd);
 struct libusb_transfer* transfer=libusb_alloc_transfer(0);
 if(!transfer) return -ENOMEM;
 libusb_fill_bulk_transfer(transfer,dev->udev,EP_OUT,(char*)&cmd_buf,31,et_write_poll_cb,adata,BULK_TIMEOUT);
 //Wait before write next poll
 tv.tv_sec=0;
 tv.tv_usec=700;
 select(0,NULL,NULL,NULL,&tv);
 r=libusb_submit_transfer(transfer);
 if(r<0)
 {
  libusb_free_transfer(transfer);
  r=-EIO;
 }
 return r;
}


static void start_finger_detection(struct fp_img_dev *dev,struct poll_data* adata)
{
   struct etss801u_dev *etdev = dev->priv;
   int r;

   if (etdev->deactivating) {
       complete_deactivation(dev);
       return;
   }

   r=et_write_poll(dev,adata);
   if(r)
   {
    fpi_imgdev_session_error(dev,r);
   }
}

/*Enroll functions (When we capturing an image)*/
void build_enroll_cmd(struct et_cmd* cmd,int stage)
{
 memset(cmd,0,sizeof(*cmd));
 switch(stage)
 {
  case 1:
  cmd->cmd=ECMD_SETREG;
  cmd->flag2=0x20;
  cmd->flag3=0x8400;
  cmd->flag4=0x74;
  cmd->flag5=0x4110;
  break;
  
  case 2:
  case 3:
  case 4:
  case 34:
  case 39:
  cmd->flag1=0x600;
  cmd->cmd=ECMD_READ_LONG;
  cmd->flag2=0x21;
  cmd->flag4=0x3;
  break;
  
  case 5:
  case 6:
  case 7:
  case 8:
  case 9:
  case 10:
  case 11:
  case 12:
  case 13:
  case 14:
  case 15:
  cmd->cmd=ECMD_SETREG;
  cmd->flag2=0x20;
  cmd->flag3=0x8400;
  cmd->flag4=0x1074;
  cmd->flag5=0x4138;
  break;
  
  case 16:
  cmd->flag1=0x1800;
  cmd->cmd=ECMD_READ_LONG;
  cmd->flag2=0x21;
  cmd->flag4=0x0C;
  break;
  
  case 17:
  case 18:
  case 19:
  case 20:
  case 21:
  case 22:
  case 23:
  case 24:
  cmd->flag1=0xF000;
  cmd->cmd=ECMD_READ_LONG;
  cmd->flag2=0x21;
  cmd->flag4=0x78;
  break;
  
  case 25:
  cmd->flag1=0x200;
  cmd->cmd=ECMD_READ_SHORT;
  cmd->flag2=0x1300;
  cmd->flag3=0x300;
  cmd->flag5=0x100;
  break;
  
  case 26:
  cmd->flag1=0x200;
  cmd->cmd=ECMD_WRITE_SHORT;
  cmd->flag2=0x2300;
  cmd->flag3=0x300;
  cmd->flag5=0x100;
  break;
  
  case 27:
  case 32:
  case 37:
  cmd->flag1=0x200;
  cmd->cmd=ECMD_READ_SHORT;
  cmd->flag2=0x1200;
  cmd->flag4=0x21;
  cmd->flag5=0x100;
  break;
  
  case 28:
  cmd->flag1=0x200;
  cmd->cmd=ECMD_READ_SHORT;
  cmd->flag2=0x1300;
  cmd->flag3=0x300;
  cmd->flag4=0x05;
  cmd->flag5=0x100;
  break;
  
  case 29:
  cmd->flag1=0x200;
  cmd->cmd=ECMD_WRITE_SHORT;
  cmd->flag2=0x2300;
  cmd->flag3=0x300;
  cmd->flag4=0x05;
  cmd->flag5=0x100;
  break;
  
  case 30:
  cmd->flag1=0x200;
  cmd->cmd=ECMD_READ_SHORT;
  cmd->flag2=0x1300;
  cmd->flag3=0x300;
  cmd->flag4=0x04;
  cmd->flag5=0x100;
  break;
  
  case 31:
  cmd->flag1=0x200;
  cmd->cmd=ECMD_WRITE_SHORT;
  cmd->flag2=0x2300;
  cmd->flag3=0x300;
  cmd->flag4=0x04;
  cmd->flag5=0x100;
  break;
  
  case 33:
  case 38:
  cmd->cmd=ECMD_SETREG;
  cmd->flag2=0x20;
  cmd->flag3=0x8400;
  cmd->flag4=0x1074;
  cmd->flag5=0x4118;
  break;
  
  case 35:
  cmd->cmd=ECMD_SETREG;
  cmd->flag2=0x8128;
  cmd->flag3=0xC400;
  cmd->flag4=0x5D0;
  cmd->flag5=0xC0C0;
  break;
  
  case 36:
  case 41:
  cmd->flag1=0x200;
  cmd->cmd=ECMD_READ_SHORT;
  cmd->flag2=0x1A00;
  cmd->flag4=0xB0;
  cmd->flag5=0x100;
  break;
  
  case 40:
  cmd->cmd=ECMD_SETREG;
  cmd->flag2=0x8128;
  cmd->flag3=0xC400;
  cmd->flag4=0x5B0;
  cmd->flag5=0xC0C0;
  break;
 }
 fill_init_cmd(cmd);
}

static void et_write_enroll_cb(struct libusb_transfer* transfer)
{
 struct init_data* adata=transfer->user_data;
 
 if(transfer->status!=LIBUSB_TRANSFER_COMPLETED)
 {
  fpi_ssm_mark_aborted(adata->ssm,-EIO);
 }
 else if(transfer->length!=transfer->actual_length)
 {
  fpi_ssm_mark_aborted(adata->ssm,-EPROTO);
 }
 else
 {
  switch(adata->init->stage)
  {
   case 26:
   case 29:
   case 31:
   if(!adata->init->rstage)
   {
    adata->init->rstage++;
    fpi_ssm_jump_to_state(adata->ssm,adata->init->ssm_state);
    goto _out;
   }
   else
   {
    adata->init->rstage=0;
    goto _wedcbn;
   }
   break;
  }
  //fp_dbg("(%02d) ->: %s",adata->init->stage,print_cmd_buf());
_wedcbn:  
  fpi_ssm_next_state(adata->ssm);
 }
_out: 
 libusb_free_transfer(transfer);
}

void free_img_buf()
{
 int i;
 
 for(i=0;i<8;i++)
 {
  if(et_img_buf[i])
  {
   g_free(et_img_buf[i]);
   et_img_buf[i]=NULL;
  }
 }
}

int et_write_enroll_data(struct fp_img_dev* dev,struct et_init* dinit,void* user_data)
{
 struct et_cmd ecmd;
 static struct init_data adata;
 int r;
 
 adata.init=dinit;
 adata.ssm=user_data;
 build_enroll_cmd(&ecmd,dinit->stage);
 struct libusb_transfer* transfer=libusb_alloc_transfer(0);
 if(!transfer) return -ENOMEM;
 libusb_fill_bulk_transfer(transfer,dev->udev,EP_OUT,(char*)&cmd_buf,31,et_write_enroll_cb,&adata,BULK_TIMEOUT);
 switch(dinit->stage)
 {
  case 5:
  cmd_buf[22]=0x38;
  break;
  
  case 6:
  cmd_buf[22]=0x50;
  break;
  
  case 7:
  case 10:
  case 14:
  cmd_buf[22]=0x68;
  break;
  
  case 8:
  cmd_buf[22]=0x78;
  break;
  
  case 9:
  case 13:
  cmd_buf[22]=0x70;
  break;
  
  case 11:
  cmd_buf[22]=0x60;
  break;
  
  case 12:
  cmd_buf[22]=0x58;
  break;
  
  case 15:
  cmd_buf[22]=0x80;
  break;
  
  /*case 16:
  et_img_buf[0]=g_malloc(6157);
  break;*/
  
  case 17:
  et_img_buf[0]=g_malloc(61453);
  break;
  
  case 18:
  et_img_buf[1]=g_malloc(61453);
  break;
  
  case 19:
  et_img_buf[2]=g_malloc(61453);
  break;
  
  case 20:
  et_img_buf[3]=g_malloc(61453);
  break;
  
  case 21:
  et_img_buf[4]=g_malloc(61453);
  break;
  
  case 22:
  et_img_buf[5]=g_malloc(61453);
  break;
  
  case 23:
  et_img_buf[6]=g_malloc(61453);
  break;
  
  case 24:
  et_img_buf[7]=g_malloc(61453);
  break;
  
  case 26:
  case 29:
  case 31:
  if(adata.init->rstage) 
  {
   libusb_fill_bulk_transfer(transfer,dev->udev,EP_OUT,(char*)&ret_buf,512,et_write_enroll_cb,&adata,BULK_TIMEOUT);
   //fp_dbg("Send back 512");
  }
  break;
 }
 r=libusb_submit_transfer(transfer);
 if(r<0)
 {
  libusb_free_transfer(transfer);
  r=-EIO;
 }
 return r;
}

int et_verify_enroll_data(struct et_init* dinit,int len)
{
 int rlen;
 
 switch(dinit->stage)
 {
  case 2:
  case 3:
  case 4:
  case 34:
  case 39:
  rlen=1536;
  if(len!=rlen) goto _eved;
  break;
  
  case 16:
  rlen=6144;
  if(len!=rlen) goto _eved;
  break;
  
  case 17:
  case 18:
  case 19:
  case 20:
  case 21:
  case 22:
  case 23:
  case 24:
  rlen=61440;
  if(len!=rlen) goto _eved;
  break;
  
  case 25:
  case 27:
  case 28:
  case 30:
  case 32:
  case 36:
  case 37:
  case 41:
  rlen=512;
  if(len!=rlen) goto _eved;
  break;
  
  //1,5 - 15,26,29,31,33,35,38,40 - skip, no data returned
 }
 return 0;
 
_eved:
 fp_dbg("Invalid answer data length: %d received, must be %d\n",len,rlen);
 return -EPROTO; 
}

static void et_read_enroll_cb(struct libusb_transfer* transfer)
{
 int r;
 int len;
 int comb=0; //Combined buffer (data+result) flag
 char* result;
 
 
 struct init_data* adata=transfer->user_data;
 if(transfer->status!=LIBUSB_TRANSFER_COMPLETED)
 {
  fpi_ssm_mark_aborted(adata->ssm,-EIO);
 }
 else
 {
  if((transfer->actual_length%2) && transfer->actual_length!=13) comb=1;
  if(comb) 
  {
   len=transfer->actual_length-13;
   //fp_dbg("<-: Read %d (combined)",transfer->actual_length-13);
  }
  else 
  {
   len=transfer->actual_length;
   //fp_dbg("<-: Read %d",transfer->actual_length);
  }
  //Data buffer received
  if(len!=13) r=et_verify_enroll_data(adata->init,len);
  else 
  {
   //fp_dbg("Result: %s",print_buf(ret_buf));
   r=et_verify_result(ret_buf);
   if(r) goto _erv;
   goto _ern;
  }
  //fp_dbg("verified");
  if(r)
  {
_erv:
   fpi_ssm_mark_aborted(adata->ssm,r);
   goto _eret;
  }
  adata->init->rstage++;
  if(!comb && len!=13) 
  {
   fp_dbg("We should not be here!");
   fpi_ssm_mark_aborted(adata->ssm,-EPROTO);
   //fpi_ssm_jump_to_state(adata->ssm,adata->init->ssm_state); //Jump back, and receive result
  }
  else
  {
   //Result received at the end of data
   switch(adata->init->stage)
   {
    case 17:
    case 18:
    case 19:
    case 20:
    case 21:
    case 22:
    case 23:
    case 24:
    //fp_dbg("Piece %d: %s",(adata->init->stage-17)+1,print_buf(et_img_buf[adata->init->stage-17]));
    result=(char*)&et_img_buf[adata->init->stage-17][len];
    break;
    
    default:
    result=(char*)&ret_buf[len];
    break;
   }
   r=et_verify_result(result);
   if(r) goto _erv;
_ern:    
   adata->init->rstage=0;
   adata->init->stage++;
   fpi_ssm_next_state(adata->ssm);
  }
 }
_eret:
 libusb_free_transfer(transfer); 
}

int et_read_enroll_data(struct fp_img_dev* dev,struct et_init* dinit,void* user_data)
{
 int r;
 static struct init_data adata;
 
 struct etss801u_dev* edev=dev->priv;
 if(edev->deactivating)
 {
  complete_deactivation(dev);
  return 0;
 }     
 adata.init=dinit;
 adata.ssm=user_data;
 struct libusb_transfer* transfer=libusb_alloc_transfer(0);
 if(!transfer) return -ENOMEM;
 switch(dinit->stage)
 {
  /*case 16:
  libusb_fill_bulk_transfer(transfer,dev->udev,EP_IN,et_img_buf[dinit->stage-16],6157,et_read_enroll_cb,&adata,BULK_TIMEOUT);
  break;*/
  
  case 17:
  case 18:
  case 19:
  case 20:
  case 21:
  case 22:
  case 23:
  case 24:
  libusb_fill_bulk_transfer(transfer,dev->udev,EP_IN,et_img_buf[dinit->stage-17],61453,et_read_enroll_cb,&adata,BULK_TIMEOUT);
  break;
  
  default:
  libusb_fill_bulk_transfer(transfer,dev->udev,EP_IN,(char*)&ret_buf,8192,et_read_enroll_cb,&adata,BULK_TIMEOUT);
  break;
 }
 r=libusb_submit_transfer(transfer);
 if(r<0)
 {
  r=-EIO;
  libusb_free_transfer(transfer);
 }
 return r;
}

static void capture_run_state(struct fpi_ssm *ssm)
{
   struct fp_img_dev *dev = ssm->priv;
   int r;

   switch(ssm->cur_state)
   {
    case 0:
    //fp_dbg("Initializing before get image data");
    memset(&einit,0,sizeof(struct et_init));
    einit.stage=1;
    case 2:
    case 4:
    case 6:
    case 8:
    case 10:
    case 12:
    case 14:
    case 16:
    case 18:
    case 20:
    case 22:
    case 24:
    case 26:
    case 28:
    case 30:
    case 32:
    //fp_dbg("Get image from scanner");
    case 34:
    case 36:
    case 38:
    case 40:
    case 42:
    case 44:
    case 46:
    goto _castate;
    case 48:
    fp_dbg("Image captured, send stop and reinit");
    case 50:
    case 52:
    case 54:
    case 56:
    case 58:
    case 60:
    case 62:
    case 64:
    case 66:
    case 68:
    case 70:
    case 72:
    case 74:
    case 76:
    case 78:
    case 80:
_castate:   
    einit.ssm_state=ssm->cur_state;
    r=et_write_enroll_data(dev,&einit,ssm);
    if(r)
    {
_cabort:
     fpi_ssm_mark_aborted(ssm,r);   
    }
    break;
    
    case 1:
    case 3:
    case 5:
    case 7:
    case 9:
    case 11:
    case 13:
    case 15:
    case 17:
    case 19:
    case 21:
    case 23:
    case 25:
    case 27:
    case 29:
    case 31:
    case 33:
    case 35:
    case 37:
    case 39:
    case 41:
    case 43:
    case 45:
    case 47:
    case 49:
    case 51:
    case 53:
    case 55:
    case 57:
    case 59:
    case 61:
    case 63:
    case 65:
    case 67:
    case 69:
    case 71:
    case 73:
    case 75:
    case 77:
    case 79:
    case 81:
    r=et_read_enroll_data(dev,&einit,ssm);
    if(r) goto _cabort;
    break;
   }
}


char* et_get_row(char* ibuf,int num)
{
 return (char*)&ibuf[768*4*num];
}

void et_assemble_image(struct fpi_img_dev* dev)
{
 //size_t sz=(61440/4)*8;
 //size_t sz=(61440*8)+6144;
 size_t sz=(192*160);
 //size_t sz=(61440/2)*8;
 //size_t sz=320*384;
 struct fp_img* img;
 int pos=0;
 char* p;

 img=fpi_img_new(sz);
 //img->flags=FP_IMG_COLORS_INVERTED;
 img->flags=FP_IMG_STANDARDIZATION_FLAGS;
 for(int i=0;i<8;i++)
 {
  for(int x=0;x<20;x++)
  {
   p=et_get_row(et_img_buf[i],x);
   for(int y=0;y<768;y+=4,pos++)
   {
    img->data[pos]=p[y];
   }
  }
  
  /*if(i)
  {
   memcpy(&img->data[pos],et_img_buf[i],61440);
   pos+=61440;
  }
  else
  {
   memcpy(&img->data[pos],et_img_buf[i],6144);
   pos+=6144;
  }*/
  /*for(int x=0;x<61440;x+=4,pos++)
  {
   img->data[pos]=et_img_buf[i][x];
  } */
  
  /*for(int x=0;x<61440;x+=2,pos++)
  {
   img->data[pos]=et_img_buf[i][x];
  }*/
  
  /*for(int x=0;x<61440;x+=2,pos++)
  {
   if(x && !(x%384))
   {
    x+=384; //continue;
   }
   img->data[pos]=et_img_buf[i][x];
  }*/
 }
 fpi_imgdev_image_captured(dev,img);
 free_img_buf();
}

static void capture_sm_complete(struct fpi_ssm *ssm)
{
   static struct poll_data adata;
   struct fp_img_dev *dev = ssm->priv;
   struct etss801u_dev *etdev = dev->priv;

   fp_dbg("");
   if (etdev->deactivating)
       complete_deactivation(dev);
   else if (ssm->error)
       fpi_imgdev_session_error(dev, ssm->error);
   else
       /*Done finger enrolling, assemble image and give it to library*/
       et_assemble_image(dev);
       fpi_imgdev_report_finger_status(dev, FALSE);
       memset(&einit,0,sizeof(struct et_init));
       adata.dev=(struct fpi_img_dev*)dev;
       adata.init=&einit;
       start_finger_detection(dev,&adata);
   fpi_ssm_free(ssm);
}

static void start_capture(struct fp_img_dev *dev)
{
   struct etss801u_dev *etdev = dev->priv;
   struct fpi_ssm *ssm;

   if (etdev->deactivating) {
       complete_deactivation(dev);
       return;
   }

   ssm = fpi_ssm_new(dev->dev, capture_run_state, ET_CAPTURE_STATES);
   fp_dbg("");
   ssm->priv = dev;
   fpi_ssm_start(ssm, capture_sm_complete);
}


/*Initialization function (when initializing device before waiting finger)*/
void build_init_cmd(struct et_cmd* cmd,int stage)
{
 memset(cmd,0,sizeof(*cmd));
 switch(stage)
 {
  case 1: //1
  case 37:
  case 38:
  cmd->flag1=0x50;
  cmd->cmd=ECMD_GETIDENT;
  cmd->flag3=0x5000;
  break;
  
  case 2: //2,4,6
  case 4:
  case 6:
  cmd->flag1=0x200;
  cmd->cmd=ECMD_READ_SHORT;
  cmd->flag2=0x1300;
  cmd->flag3=0x300;
  cmd->flag4=0x07;
  cmd->flag5=0x100;
  break;
  
  case 3: //3,5,11,15
  case 5:
  case 11:
  case 15:
  case 36:
  case 44:
  case 50:
  cmd->flag1=0x200;
  cmd->cmd=ECMD_WRITE_SHORT;
  cmd->flag2=0x2300;
  cmd->flag3=0x300;
  cmd->flag4=0x04;
  cmd->flag5=0x100;
  break;

  case 7:
  cmd->flag1=0x200;
  cmd->cmd=ECMD_WRITE_SHORT;
  cmd->flag2=0x2300;
  cmd->flag3=0x1000;
  cmd->flag4=0x0F;
  cmd->flag5=0x100;
  break;
  
  case 8:
  cmd->flag1=0x20;
  cmd->cmd=ECMD_GETNSTRING;
  cmd->flag3=0xFF01;
  cmd->flag6=0x01;
  break;
  
  case 9:
  cmd->flag1=0x20;
  cmd->cmd=ECMD_GETNSTRING;
  cmd->flag3=0x9F01;
  cmd->flag6=0x301;
  break;
  
  case 10:
  case 14:
  case 35:
  case 43:
  case 49:
  cmd->flag1=0x200;
  cmd->cmd=ECMD_READ_SHORT;
  cmd->flag2=0x1300;
  cmd->flag3=0x300;
  cmd->flag4=0x04;
  cmd->flag5=0x100;
  break;
  
  case 12:
  case 41:
  case 47:
  cmd->flag1=0x200;
  cmd->cmd=ECMD_READ_SHORT;
  cmd->flag2=0x1300;
  cmd->flag3=0x300;
  cmd->flag4=0x05;
  cmd->flag5=0x100;
  break;
  
  case 13:
  case 42:
  case 48:
  cmd->flag1=0x200;
  cmd->cmd=ECMD_WRITE_SHORT;
  cmd->flag2=0x2300;
  cmd->flag3=0x300;
  cmd->flag4=0x05;
  cmd->flag5=0x100;
  break;
  
  case 16:
  case 45:
  case 51:
  case 56:
  cmd->flag1=0x200;
  cmd->cmd=ECMD_READ_SHORT;
  cmd->flag2=0x1200;
  cmd->flag4=0x21;
  cmd->flag5=0x100;
  break;
  
  case 17:
  case 46:
  cmd->flag1=0x200;
  cmd->cmd=ECMD_WRITE_SHORT;
  cmd->flag2=0x2200;
  cmd->flag4=0x21;
  cmd->flag5=0x100;
  break;
  
  case 18:
  case 52:
  case 57:
  cmd->cmd=ECMD_SETREG;
  cmd->flag2=0x20;
  cmd->flag3=0x8400;
  cmd->flag4=0x1074;
  cmd->flag5=0x4118;
  break;
  
  case 19:
  cmd->cmd=ECMD_SETREG;
  cmd->flag2=0x20;
  cmd->flag3=0x8400;
  cmd->flag4=0x74;
  cmd->flag5=0x4110;
  break;
  
  case 20:
  case 22:
  case 24:
  case 26:
  case 28:
  case 30:
  case 32:
  case 34:
  case 53:
  case 58:
  cmd->flag1=0x600;
  cmd->cmd=ECMD_READ_LONG;
  cmd->flag2=0x21;
  cmd->flag4=0x3;
  break;
  
  case 21:
  cmd->cmd=ECMD_SETREG;
  cmd->flag2=0x20;
  cmd->flag3=0x8400;
  cmd->flag4=0x78;
  cmd->flag5=0x4110;
  break;
  
  case 23:
  cmd->cmd=ECMD_SETREG;
  cmd->flag2=0x20;
  cmd->flag3=0x8400;
  cmd->flag4=0x1078;
  cmd->flag5=0x4118;
  break;
  
  case 25:
  cmd->cmd=ECMD_SETREG;
  cmd->flag2=0x20;
  cmd->flag3=0x8400;
  cmd->flag4=0x2078;
  cmd->flag5=0x4120;
  break;
  
  case 27:
  cmd->cmd=ECMD_SETREG;
  cmd->flag2=0x20;
  cmd->flag3=0x8400;
  cmd->flag4=0x3078;
  cmd->flag5=0x4128;
  break;
  
  case 29:
  cmd->cmd=ECMD_SETREG;
  cmd->flag2=0x20;
  cmd->flag3=0x8400;
  cmd->flag4=0x4078;
  cmd->flag5=0x4130;
  break;
  
  case 31:
  cmd->cmd=ECMD_SETREG;
  cmd->flag2=0x20;
  cmd->flag3=0x8400;
  cmd->flag4=0x5078;
  cmd->flag5=0x4138;
  break;
  
  case 33:
  cmd->cmd=ECMD_SETREG;
  cmd->flag2=0x20;
  cmd->flag3=0x8400;
  cmd->flag4=0x6078;
  cmd->flag5=0x4140;
  break;

  case 39:
  cmd->flag1=0x200;
  cmd->cmd=ECMD_READ_SHORT;
  cmd->flag2=0x1300;
  cmd->flag3=0x300;
  cmd->flag5=0x100;
  break;

  case 40:
  cmd->flag1=0x200;
  cmd->cmd=ECMD_WRITE_SHORT;
  cmd->flag2=0x2300;
  cmd->flag3=0x300;
  cmd->flag5=0x100;
  break;
  
  case 54:
  cmd->cmd=ECMD_SETREG;
  cmd->flag2=0x8128;
  cmd->flag3=0xC400;
  cmd->flag4=0x5D0;
  cmd->flag5=0xC0C0;
  break;
  
  case 55:
  cmd->flag1=0x200;
  cmd->cmd=ECMD_READ_SHORT;
  cmd->flag2=0x1A00;
  cmd->flag4=0xB0;
  cmd->flag5=0x100;
  break;
  
  case 59:
  cmd->cmd=ECMD_SETREG;
  cmd->flag2=0x8128;
  cmd->flag3=0xC400;
  cmd->flag4=0x5B0;
  cmd->flag5=0xC0C0;
  break;
      
  default:
  break;
 }
}

void fill_init_cmd(struct et_cmd* cmd)
{
 char* p;
 
 p=&cmd_buf[8];
 memcpy(p,&cmd->flag1,2);
 p+=6;
 memcpy(p,&cmd->cmd,2);
 p+=2;
 memcpy(p,&cmd->flag2,2);
 p+=2;
 memcpy(p,&cmd->flag3,2);
 p+=2;
 memcpy(p,&cmd->flag4,2);
 p+=2;
 memcpy(p,&cmd->flag5,2);
 p+=2;
 memcpy(p,&cmd->flag6,2);
 p+=2;
}

int et_write_init(struct fp_img_dev* dev,struct et_init* dinit,void* user_data)
{
 struct et_cmd ecmd;
 static struct init_data adata;
 int r;
 
 adata.init=dinit;
 adata.ssm=user_data;
 build_init_cmd(&ecmd,dinit->stage);
 fill_init_cmd(&ecmd);
 //Send command to device
 struct libusb_transfer* transfer=libusb_alloc_transfer(0);
 if(!transfer) return -ENOMEM;
 switch(dinit->stage)
 {
  case 3:
  case 36:
  ret_buf[0]=0x48;
  goto _evi_w;
  case 5:
  case 7:
  ret_buf[0]=0x0C;
  goto _evi_w;
  case 11:
  case 40:
  case 42:
  case 48:
  case 50:
  goto _evi_w;
  case 13:
  ret_buf[0]=0x23;
  goto _evi_w;
  case 15:
  case 44:
  ret_buf[0]=0x4D;
  goto _evi_w;
  case 17:
  ret_buf[0]=0x52;
  goto _evi_w;
  case 46:
  ret_buf[0]=0x42;
_evi_w:  
  if(!dinit->rstage)
   libusb_fill_bulk_transfer(transfer,dev->udev,EP_OUT,(char*)&cmd_buf,31,et_write_init_cb,&adata,BULK_TIMEOUT);
  else 
  { 
   //fp_dbg("ret_buf: 0x%02hhX",ret_buf[0]);
   libusb_fill_bulk_transfer(transfer,dev->udev,EP_OUT,(char*)&ret_buf,512,et_write_init_cb,&adata,BULK_TIMEOUT);
  }
  break;
  
  default:
  libusb_fill_bulk_transfer(transfer,dev->udev,EP_OUT,(char*)&cmd_buf,31,et_write_init_cb,&adata,BULK_TIMEOUT);
  break;
 }
 //fp_dbg("->: %s",print_cmd_buf());
 r=libusb_submit_transfer(transfer);
 if(r<0)
 {
  libusb_free_transfer(transfer);
  r=-EIO;
 }
 return r;
}

static void et_write_init_cb(struct libusb_transfer* transfer)
{
 struct init_data* adata=transfer->user_data;
 if(transfer->status!=LIBUSB_TRANSFER_COMPLETED)
 {
  fpi_ssm_mark_aborted(adata->ssm,-EIO);
 }
 else if(transfer->length!=transfer->actual_length)
 {
  fpi_ssm_mark_aborted(adata->ssm,-EPROTO);
 }
 else
 {
  switch(adata->init->stage)
  {
   case 3:
   case 5:
   case 7:
   case 11:
   case 13:
   case 15:
   case 17:
   case 36:
   case 40:
   case 42:
   case 44:
   case 46:
   case 48:
   case 50:
   if(!adata->init->rstage)
   {
    //fp_dbg("(%02d) ->: %s",adata->init->stage,print_cmd_buf());
    adata->init->rstage++;
    fpi_ssm_jump_to_state(adata->ssm,adata->init->ssm_state);
    goto _wicb;
   }
   else
   {
    adata->init->rstage=0;
    //fp_dbg("(%02d) ->: RETURN BUF %d",adata->init->stage,transfer->actual_length);
    goto _wicbn;
   }
   break;
  }
  //fp_dbg("(%02d) ->: %s",adata->init->stage,print_cmd_buf());
_wicbn:  
  fpi_ssm_next_state(adata->ssm);
 }
_wicb: 
 libusb_free_transfer(transfer);
}

int et_read_answer(struct fp_img_dev* dev,struct et_init* dinit,void* user_data)
{
 int r;
 static struct init_data adata;
 
 adata.init=dinit;
 adata.ssm=user_data;
 struct libusb_transfer* transfer=libusb_alloc_transfer(0);
 if(!transfer) return -ENOMEM;
 libusb_fill_bulk_transfer(transfer,dev->udev,EP_IN,(char*)&ret_buf,8192,et_read_answer_cb,&adata,BULK_TIMEOUT);
 r=libusb_submit_transfer(transfer);
 if(r<0)
 {
  r=-EIO;
  libusb_free_transfer(transfer);
 }
 return r;
}

char* print_cmd_buf()
{
 static char result[512];
 char* p;
 
 int i;
 p=(char*)&result[0];
 for(i=8;i<26;i++)
 {
  if(i!=8)
  {
   sprintf(p,",");
   p++;
   p[0]=0;
  }
  sprintf(p,"0x%02hhX",cmd_buf[i]);
  p+=4;
  p[0]=0;
 }
 return (char*)result;
}

char* print_buf(char* buf)
{
 static char result[512];
 char* p;
 
 int i;
 p=(char*)&result[0];
 for(i=0;i<13;i++)
 {
  if(i)
  {
   sprintf(p,",");
   p++;
   p[0]=0;
  }
  sprintf(p,"0x%02hhX",buf[i]);
  p+=4;
  p[0]=0;
 }
 return (char*)result;
}


static void et_read_answer_cb(struct libusb_transfer* transfer)
{
 int r;
 int len;
 int comb=0; //Combined buffer (answer+data) flag
 char* result;
 
 
 struct init_data* adata=transfer->user_data;
 if(transfer->status!=LIBUSB_TRANSFER_COMPLETED)
 {
  fpi_ssm_mark_aborted(adata->ssm,-EIO);
 }
 else
 {
  if((transfer->actual_length%2) && transfer->actual_length!=13) comb=1;
  if(comb) 
  {
   len=transfer->actual_length-13;
   //fp_dbg("<-: Read %d (combined)",transfer->actual_length-13);
  }
  else 
  {
   len=transfer->actual_length;
   //fp_dbg("<-: Read %d",transfer->actual_length);
  }
  if(!adata->init->rstage)
  {
   //Data buffer received
   if(len!=13) r=et_verify_answer(adata->init,len);
   else 
   {
    //fp_dbg("Result: %s",print_buf(ret_buf));
    r=et_verify_result(ret_buf);
    if(r) goto _erv;
    goto _ern;
   }
   //fp_dbg("verified");
   if(r)
   {
_erv:
    fpi_ssm_mark_aborted(adata->ssm,r);
    goto _eret;
   }
   adata->init->rstage++;
   if(!comb && len!=13) fpi_ssm_jump_to_state(adata->ssm,adata->init->ssm_state); //Jump back, and receive result
   else
   {
    //Result received at the end of data
    result=(char*)&ret_buf[len];
    r=et_verify_result(result);
    if(r) goto _erv;
_ern:    
    adata->init->rstage=0;
    adata->init->stage++;
    fpi_ssm_next_state(adata->ssm);
   }
  }
  else
  {
   //Result received
   if(len!=13) { r=-EPROTO; goto _erv; }
   r=et_verify_result(ret_buf);
   if(r) goto _erv;
   adata->init->stage++;
   adata->init->rstage=0;
   fpi_ssm_next_state(adata->ssm);
  }
 }
_eret: 
 libusb_free_transfer(transfer);
}

int et_verify_answer(struct et_init* dinit,int len)
{
 int rlen;
 
 switch(dinit->stage)
 {
  case 1:
  case 37:
  case 38:
  rlen=80;
  if(len!=rlen) goto _eva;
  //fp_dbg("verify: %d",dinit->ssm_state);
  break;
  
  case 2:
  case 4:
  case 6:
  case 10:
  case 12:
  case 14:
  case 16:
  case 35:
  case 39:
  case 41:
  case 43:
  case 45:
  case 47:
  case 49:
  case 51:
  case 55:
  case 56:
  //fp_dbg("verify: len %d",len);
  rlen=512;
  if(len!=rlen) goto _eva;
  break;
  
  /*3,5,7,11,13,15,17,18,19,21,23,25,27,29,31,33,36,40,42,44,46,48,50,52,54,57,59 - only result
  */
  
  case 8:
  case 9:
  rlen=32;
  if(len!=rlen) goto _eva;
  break;
  
  case 20:
  case 22:
  case 24:
  case 26:
  case 28:
  case 30:
  case 32:
  case 34:
  case 53:
  case 58:
  rlen=1536;
  if(len!=rlen) goto _eva;
  break;
  
 }
 return 0;
 
_eva:
 fp_dbg("Invalid answer data length: %d received, must be %d\n",len,rlen);
 return -EPROTO; 
}

int et_verify_result(char* result)
{
 char vresult[]={0,0,0,0,0};
 char ah[]={ET_ANS_HDR};
 char pd[]={ET_BUF_PDATA};
 char* v;
 
 v=(char*)&result[0];
 if(memcmp(v,ah,4)) goto _evr;
 v=(char*)&result[4];
 if(memcmp(v,pd,4)) goto _evr;
 v=(char*)&result[8];
 if(memcmp(v,vresult,5)) goto _evr;
 return 0;
 
_evr:
 fp_dbg("Invalid command result: %s",print_buf(result));
 return -EPROTO; 
}

static void activate_run_state(struct fpi_ssm *ssm)
{
   int r;
   struct fp_img_dev *dev = ssm->priv;

   /* Activation process of SS801U is not understandable at
   the moment. Simply repeats captured protocol.
   And, I don't know how to enter in initial state without device reset
    */
   switch (ssm->cur_state)
   {
    case 0:
    //Reset device
    libusb_release_interface(dev->udev,0);
    r=libusb_reset_device(dev->udev);
    if(r<0)
    {
_eop:
     fpi_ssm_mark_aborted(ssm,r);
     return;
    }
    r=libusb_claim_interface(dev->udev,0);
    if(r) goto _eop;
    memset(&einit,0,sizeof(struct et_init));
    einit.stage=1;
    case 2:
    case 4:
    case 6:
    case 8:
    case 10:
    case 12:
    case 14:
    case 16:
    case 18:
    case 20:
    case 22:
    case 24:
    case 26:
    case 28:
    case 30:
    case 32:
    case 34:
    case 36:
    case 38:
    case 40:
    case 42:
    case 44:
    case 46:
    case 48:
    case 50:
    case 52:
    case 54:
    case 56:
    case 58:
    case 60:
    case 62:
    case 64:
    case 66:
    case 68:
    case 70:
    case 72:
    case 74:
    case 76:
    case 78:
    case 80:
    case 82:
    case 84:
    case 86:
    case 88:
    case 90:
    case 92:
    case 94:
    case 96:
    case 98:
    case 100:
    case 102:
    case 104:
    case 106:
    case 108:
    case 110:
    case 112:
    case 114:
    case 116:
    einit.ssm_state=ssm->cur_state;
    r=et_write_init(dev,&einit,ssm);
    if(r)
    {
_mabort:
     fpi_ssm_mark_aborted(ssm,r);
    }
    break;
    
    case 1:
    case 3:
    case 5:
    case 7:
    case 9:
    case 11:
    case 13:
    case 15:
    case 17:
    case 19:
    case 21:
    case 23:
    case 25:
    case 27:
    case 29:
    case 31:
    case 33:
    case 35:
    case 37:
    case 39:
    case 41:
    case 43:
    case 45:
    case 47:
    case 49:
    case 51:
    case 53:
    case 55:
    case 57:
    case 59:
    case 61:
    case 63:
    case 65:
    case 67:
    case 69:
    case 71:
    case 73:
    case 75:
    case 77:
    case 79:
    case 81:
    case 83:
    case 85:
    case 87:
    case 89:
    case 91:
    case 93:
    case 95:
    case 97:
    case 99:
    case 101:
    case 103:
    case 105:
    case 107:
    case 109:
    case 111:
    case 113:
    case 115:
    case 117:
    einit.ssm_state=ssm->cur_state;
    r=et_read_answer(dev,&einit,ssm);
    if(r) goto _mabort;
    break;
   
        }
}

/* jump to finger detection */
static void activate_sm_complete(struct fpi_ssm *ssm)
{
   static struct poll_data adata;
   
   struct fp_img_dev *dev = ssm->priv;
   fp_dbg("status %d", ssm->error);
   fpi_imgdev_activate_complete(dev, ssm->error);

   if (!ssm->error)
       memset(&einit,0,sizeof(struct et_init));
       adata.dev=(struct fpi_img_dev*)dev;
       adata.init=&einit;
       start_finger_detection(dev,&adata);
   fpi_ssm_free(ssm);
}

static int dev_activate(struct fp_img_dev *dev, enum fp_imgdev_state state)
{
   struct fpi_ssm *ssm = fpi_ssm_new(dev->dev, activate_run_state,ET_ACTIVATE_STATES);
   ssm->priv = dev;
   fpi_ssm_start(ssm, activate_sm_complete);
   return 0;
}

static void dev_deactivate(struct fp_img_dev *dev)
{
   struct etss801u_dev *etdev = dev->priv;
   etdev->deactivating = TRUE;
}

static void complete_deactivation(struct fp_img_dev *dev)
{
   struct etss801u_dev *etdev = dev->priv;
   fp_dbg("");
   etdev->deactivating = FALSE;
   fpi_imgdev_deactivate_complete(dev);
}

static int dev_init(struct fp_img_dev *dev, unsigned long driver_data)
{
   int r;

   r = libusb_claim_interface(dev->udev, 0);
   if (r < 0) {
       fp_err("could not claim interface 0");
       return r;
   }

   //dev->dev->nr_enroll_stages=3;

   dev->priv = g_malloc0(sizeof(struct etss801u_dev));
   fpi_imgdev_open_complete(dev, 0);
   return 0;
}

static void dev_deinit(struct fp_img_dev *dev)
{
   g_free(dev->priv);
   libusb_release_interface(dev->udev, 0);
   fpi_imgdev_close_complete(dev);
}

static const struct usb_id id_table[] = {
   { .vendor = 0x1c7a, .product = 0x0801 }, /* EgisTec SS801U */
   { 0, 0, 0, },
};

struct fp_img_driver etss801u_driver = {
   .driver = {
       .id = 19, //I take first free number, but at final, developer's decision needed.
       .name = FP_COMPONENT,
       .full_name = "EgisTec SS801U",
       .id_table = id_table,
       .scan_type = FP_SCAN_TYPE_SWIPE,
   },
   .flags = 0,
   //.img_height = 640,
   .img_height = 160,
   .img_width = 192,
   //.img_width = 768, //Still in doubt, maybe 192
        //.img_width = 192,
        //.img_width = 384,

        .bz3_threshold = 19,
   
   .open = dev_init,
   .close = dev_deinit,
   .activate = dev_activate,
   .deactivate = dev_deactivate,
};


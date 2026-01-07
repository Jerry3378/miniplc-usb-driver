// SPDX-License-Identifier: GPL-2.0
/*
 * USB Skeleton driver - 2.2
 *
 * Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com)
 * Copyright (C) 2026 JaemoPark (packjemo@gmail.com)
 *
 * This driver is based on the 2.6.3 version of drivers/usb/usb-skeleton.c
 * and has been modified for the miniPLC project.
 */

#include <linux/kernel.h>	//커널 개발시 쓰이는 기본 함수(printk, KERNER_INFO, min,max 매크로 사용)
#include <linux/errno.h>	//에러 코드 정의 (-EINVAL, -EFAULT, -ENOMEM, -EBUSY 등). 커널 함수들이 실패할 때 이 값들을 리턴.
#include <linux/slab.h>		//커널 메모리 할당 관련 (kmalloc(), kfree(), kzalloc()). USB 드라이버에서 패킷 버퍼, 디스크립터 구조체 등을 동적으로 생성할 때 사용.
#include <linux/module.h>	//모듈 관련 매크로/함수 (module_init, module_exit, MODULE_LICENSE, MODULE_AUTHOR). 드라이버를 insmod/rmmod 할 수 있도록 지원.
#include <linux/kref.h>		//참조 카운트(refcount) 관리 구조체. USB 장치 객체 같은 걸 여러 함수가 공유할 때, 안전하게 해제하기 위해 참조 수를 관리.
#include <linux/uaccess.h>	//유저 공간 <-> 커널 공간 데이터 복사 함수 (copy_to_user, copy_from_user). 드라이버에서 read/write 시스템콜을 구현할 때 꼭 필요.
#include <linux/usb.h>		//USB 관련 구조체와 함수 정의. struct usb_device, struct usb_interface, usb_register_driver, usb_bulk_msg 등이 여기 있음. USB 드라이버의 핵심.
#include <linux/mutex.h>	//뮤텍스 락 제공 (struct mutex, mutex_lock, mutex_unlock). 드라이버 내부에서 여러 프로세스가 동시에 장치를 접근하지 않도록 보호.
#include <linux/printk.h>   // hexdump 형식 헤더

/* driver 내부 버퍼 관련 변수*/
#define BUFFER_SIZE 1024 //  rx_buffer용 사이즈 (2의 10 제곱)

/* 드라이버와 매칭시킬 디바이스 벤더 ID와 ProductID를 설정한다. */
#define USB_PROJECT_VENDOR_ID	0x03eb
#define USB_PROJECT_PRODUCT_ID	0x206c
#define USB_INTERFACE_CLASS 0xff		//디바이스 인터페이스 번호(부호 없는 8비트 형식).

/* 최대 전송 길이입니다. */
#define MAX_TRANSFER		64  // atmega32u4같은 경우는 full speed여서 urb당 64바이트임
#define WRITES_IN_FLIGHT	5   // 한번에 write할때 보낼 urb 갯수

/* 해당 드라이버와 매칭할 USB테이블을 할당한다.  우리가 사용할 atmega32u4같은 경우 
	이미 드라이버들이 있기 때문에 USB_DEVICES_INTERFACE_CLASS 매크로를 통해 usb_device_id 구조체에 
	인터페이스 클래스를 추가함으로써 새로운 인터페이스를 사용하는 드라이버로 만들 예정이다. 
	USB_DEVICE_INTERFACE_CLASS 같은 경우 벤더, product 아이디와 추가로 INTERFACE CLASS 값을 요구한다.*/

    static const struct usb_device_id usb_device_table[] = {
	//Vendor, Product ID와 Interface 넘버를 구조체 배열 형식으로 등록한다.
	{ USB_DEVICE_INTERFACE_CLASS(USB_PROJECT_VENDOR_ID, USB_PROJECT_PRODUCT_ID,0xff) },
	{ }					/* Terminating entry */
};

struct protocol_header {
    __u8 stx;
    __u8 len;
    __u8 unit;
    __u8 cmd;
    __u16 address;
    __u8 payload;
} __attribute__((packed));

//probe단계 중 usb_find_interface함수에서 my_usb_driver가 호출됨으로 여기서 미리 protocol을 선언한다.
//함수 정의를 옮기기에는 my_sub_driver 구조체도 probe, disconnect를 멤버로 가지기엔 protocol선언이 바람직해 보인다.
static struct usb_driver my_usb_driver;

/* USB 드라이버에서 사용할 디바이스에 특정 정보를 담기 위한 구조체*/
struct usb_info {
    /*usb디바이스 관련 정보*/
    struct usb_device *udev;    //usb 디바이스 구조체 포인터
    struct usb_interface *interface;    //usb 인터페이스 구조체 포인터
    struct usb_anchor submitted;    //usb anchor 구조체(제출한 URB관리를 위한)
    struct urb *bulk_in_urb;    //usb urb 구조체 포인터(bulk_in)
    struct semaphore limit_sem;     // write를 할때 urb의 갯수를 제한해둠

    /*엔드포인트 관련 변수들(bulk endpoint관련) */
    //bulk_in
    unsigned char *bulk_in_buffer; //bulk in 데이터를 가리키는 버퍼 포인터(커널 버퍼 포인터)

    size_t bulk_in_size;        //bulk in 엔드포인트 최대 패킷 크기(한 번에 받을 수 있는 최대 크기)
    size_t bulk_in_filled;     //버퍼 안에 있는 데이터 크기(지금 버퍼 안에 쌓여 있는 데이터 크기)
    size_t bulk_in_copied;      //user space로 복사된 bulk_in 엔드포인트 크기(유저 공간으로 복사된 데이터 크기)
    __u8 bulk_in_endpointAddr; //bulk in 엔드포인트 주소

    //bulk out
    __u8 bulk_out_endpointAddr; //bulk out 엔드포인트 주소(bulk out은 write를 통해 장치에 보내주기만 하면 됨)
    unsigned char *bulk_out_buffer;     // bulk out 버퍼입니다.

    /*커널 관련 변수*/
    struct mutex io_mutex; //뮤텍스 구조체(임계영역 보호용)
    spinlock_t err_lock;	//read,write할때 error을 확인하기 위한 키
    struct kref kref;	//kernel reference, 즉 커널 참조 변수 얼마나 많이 장치에 연결되어있는지를 나타냄
    wait_queue_head_t bulk_in_wait;		//read가 끝날때 까지 기다릴 변수
    bool ongoing_read;                  //read의 현재 상태를 나타냄, false: 새로운 데이터를 읽을 준비가 됨. true: 현재 버스(USB) 위에서 데이터가 오고 있는 중이니 기다려야 함.
    unsigned long disconnected:1;        //disconnected된 상태를 알려줄 상태 flag변수 (변수이름뒤에:1은 해당 변수중 1비트만 쓰겠다는 의미)
    int errors;                          // error에 대한 상태

    // 커널 내부 링버퍼 관련 버퍼
    unsigned int head;       // 생성자
    unsigned int tail;       // 소비자    
    __u8 rx_buffer[BUFFER_SIZE];       // USB 엔드포인트로부터 수신된 데이터를 저장하는 수신(RX) 버퍼
    struct mutex buf_mutex;                 // 링 버퍼용 뮤택스

    // 프로토콜 관련 변수
    struct protocol_header header;
};

//해당 매크로 함수는 매개변수 d를 멤버로 가지는 구조체의 주소를 찾기 위한 매크로 함수입니다. 
#define to_usb_dev(d) container_of(d, struct usb_info, kref)

/* usb정보를 delete하는 함수입니다. 
   kref값이 0이 되면 자동으로 호출되어서 
   할당된 usb 정보를 free 시키고 디바이스를 free합니다
   커널은 함수의 순서가 중요해서 delete가 먼저 있어야지 release, disconnect에서도 사용 가능합니다.
   */
static void usb_delete(struct kref *kref){
    struct usb_info *dev = to_usb_dev(kref);

    printk("[USB DELETE][usb_delete] : kref reached %d!, deleting device", kref_read(&dev->kref)); 

    //urb해제합니다(disconnect단계에선 단지 정보만 삭제했음).
    usb_free_urb(dev->bulk_in_urb);
    printk("[USB DELETE][usb_delete] : success to free urb\n");
    //interface 레퍼런스를 1 감소하고 0이면 interface를 free함
    usb_put_intf(dev->interface);
    printk("[USB DELETE][usb_delete] : success to free interface\n");
    // device 레퍼런스를 1 감소하고 0이면 device를 free함
    usb_put_dev(dev->udev);
    printk("[USB DELETE][usb_delete] : success to free udev\n");
    kfree(dev->bulk_in_buffer);     //bulk를 free함
    kfree(dev);         //디바이스를 free함
}


/* usb 파일을 open할때 호출되는 콜백 함수이다.
 * 매개변수로는 inode와 file 구조체가 있다.
 * inode는 리눅스 커널에서 inode는 파일의 메타데이터를 담는 구조체
 * 실제 파일 내용은 struct file에서 다루고, inode는 파일 자체의 속성, 타입, 장치 정보, 참조 등을 담고 있음.
*/
static int usb_open(struct inode *inode, struct file *file) 
{
    //dev는 디바이스 정보와 디바이스 제어(urb 송/수신) 위주임
    struct usb_info *dev;
    //interface는 주로 usbcore단에서 interface, endpoint제어를 위주로 함
    struct usb_interface *interface;

    //마이너 번호
    int subminor;
    int retval = 0;

    //inode 구조체를 통해서 마이너 번호를 얻음
    subminor = iminor(inode);

    //minor 번호로 dev 구조체를 찾기 위한 interface 조회
    //사용 이유 : open/read/write 같은 시점에서, 유저가 /dev/usb0 열었을 때
    //inode -> minor 번호 -> usb_driver -> 인터페이스 찾음
    interface = usb_find_interface(&my_usb_driver, subminor);

    //찾지 못할 시
    if(!interface) {
        pr_err("%s - error, can't find device for minor %d\n",
			__func__, subminor);
		retval = -ENODEV;
		goto exit;
    }

    dev_info(&interface->dev, "[USB OPEN][my_usb_open] : success to find interface!\n");

    //struct usb_interface에는 driver_data용 private 포인터를 get함
    //(usb_set_intfdata시 여기에 저장된 포인터를 장치를 사용하기 위해 불러옴)
    dev = usb_get_intfdata(interface);

    if (!dev) {
		retval = -ENODEV;
		goto exit;
	}

    dev_info(&interface->dev, "[USB OPEN][my_usb_open] : success to get interface data!\n");

    //참조 카운트 1증가
    kref_get(&dev->kref);

    dev_info(&interface->dev, "[USB OPEN][my_usb_open] : success to open! kref status : %d\n", kref_read(&dev->kref));        //kref_read를 통해서 읽기

    //파일에 디바이스 등록
    file->private_data = dev;

exit:
    return retval;
    

}

/* usb 파일을 close할때 호출되는 콜백 함수이다.
 * 매개변수로는 inode와 file 구조체가 있다.
 * inode는 리눅스 커널에서 inode는 파일의 메타데이터를 담는 구조체 즉 물리적인 파일
 * 실제 파일 내용은 struct file에서 다루고, inode는 파일 자체의 속성, 타입, 장치 정보, 참조 등을 담고 있음.
*/
static int usb_release(struct inode *inode, struct file *file) {
    
    
    struct usb_info *dev;       // usb 지역 구조체

    dev = file->private_data;       //실제 interface의 private에 저장된 로컬 구조체
    if (dev == NULL) {
        return -ENODEV;
    }

    printk("[USB RELEASE][usb_release] : success to find usb_dev\n");

    // close를 할때 kref값을 1 줄임
    kref_put(&dev->kref, usb_delete);

    printk("[USB RELEASE][usb_release] : success close file, kref changes %d -> %d\n", kref_read(&dev->kref)+1, kref_read(&dev->kref));
    return 0;
}

static int dequeue(struct urb *urb) 
{
    struct usb_info *dev = urb->context;

    mutex_lock(&dev->buf_mutex);

    // TODO : REad 구현
    // if ((dev->tail) % BUFFER_SIZE)

    return 0;
}

// ring buffer에 read callback 함수로부터 받은 데이터를 복사할 함수
static int enqueue(struct urb *urb)
{
    struct usb_info *dev = urb->context;

    mutex_lock(&dev->buf_mutex);

    // 링버퍼 full 체크
    if ((dev->head + 1) % BUFFER_SIZE == dev->tail) {
        pr_err("ring buffer full!\n");
        return -ENOMEM;
    }

    // 데이터 복사
    memcpy(dev->rx_buffer[dev->head], dev->bulk_in_buffer, urb->actual_length);

    // head 이동 buffer size로 나머지 연산을 해야함
    dev->head = (dev->head + 1) % BUFFER_SIZE;

    mutex_unlock(&dev->buf_mutex);

    return 0;
}


// 디바이스로부터 요청한 read_bulk_urb가 성공적으로 올때 호출됩니다.
static void usb_read_callback(struct urb *urb) {
    struct usb_info *dev;
    unsigned long flags;

    // usb_fill_bulk_urb에서 context는 device입니다
    dev = urb->context;
    printk("[USB_READ_CALLBACK][usb_read_callback] success to call read_callback\n");

    // 현재 CPU의 인터럽트를 끄고 인터럽트를 수행한다(인터럽트 핸들러 역활과 비슷함)
    spin_lock_irqsave(&dev->err_lock, flags);
    // "URB(USB Request Block)의 연결을 끊을 때 발생하는 상태값은 하드웨어 에러로 취급하지 않겠다
    if (urb->status) {
        if (!(urb->status == -ENOENT ||
              urb->status == -ECONNRESET ||
              urb->status == -ESHUTDOWN))
              dev_err(&dev->interface->dev,
				"%s - nonzero write bulk status received: %d\n",
				__func__, urb->status);
        
        dev->errors = urb->status;
    } else {
        dev->bulk_in_filled = urb->actual_length;
        printk("[USB_READ_CALLBACK][usb_read_callback] success set bulk_in_filled : %d\n",dev->bulk_in_filled);
    }

    // 읽기 종료
    dev->ongoing_read = 0;
    spin_unlock_irqrestore(&dev->err_lock, flags);

    
    print_hex_dump(KERN_INFO, "rx: ",
               DUMP_PREFIX_OFFSET,
               16, 1,
               urb->transfer_buffer,
               urb->actual_length,
               true);
    
    

    // bulk-in 전송이 완료되었으므로, 블로킹(Blocking)된 읽기 작업을 실행 상태로 전환합니다.
    wake_up_interruptible(&dev->bulk_in_wait);
}

static int usb_do_read_io(struct usb_info *dev, size_t count) {
    
    int retval;

    // 읽을 준비
    usb_fill_bulk_urb(dev->bulk_in_urb,      //urb
                dev->udev,                  //device
                usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr),      //endpoint pipe
                dev->bulk_in_buffer,            // 전송할 버퍼
                min(dev->bulk_in_size, count),       // 버퍼 크기
                usb_read_callback,           // 호출할 call back 함수
                dev);
    printk("[USB_DO_READ_IO][usb_do_read_io] success to fill bulk urb\n");

    // urb 읽는 중 설정
    spin_lock_irq(&dev->err_lock);
    dev->ongoing_read = 1;
    spin_unlock_irq(&dev->err_lock);
    printk("[USB_DO_READ_IO][usb_do_read_io] success to set ongoing_read : 1\n");

    // bulk in urb 전송, 전송할 데이터가 없음
    dev->bulk_in_filled = 0;
    dev->bulk_in_copied = 0;

    // 장치에게 보냄 submit함
    retval = usb_submit_urb(dev->bulk_in_urb, GFP_KERNEL);
    printk("[USB_DO_READ_IO][usb_do_read_io] success to submit urb\n");

    // 오류 시
    if (retval < 0) {
		dev_err(&dev->interface->dev,
			"%s - failed submitting read urb, error %d\n",
			__func__, retval);
		retval = (retval == -ENOMEM) ? retval : -EIO;
		spin_lock_irq(&dev->err_lock);
		dev->ongoing_read = 0;          // ongoing_read가 0이면 읽기 완료
		spin_unlock_irq(&dev->err_lock);
	}

    return retval;
}

static ssize_t usb_read(struct file *file, char *buffer, size_t count, loff_t *ppos) {
    
    // usb 구조체 변수
    struct usb_info *dev;
    int retval;
    bool ongoing_io;

    // file(interface)에서 저장한 로컬 구조체(usb_info)를 불러옴
    dev = file->private_data;

    // 읽을 길이가 0일시 종료
    if (!count) {
        return 0;
    }

    printk("[USB_READ][usb_read] : success to call read function!!\n");


    // mutex lock과 같이 임계영역을 기다리되 event 즉 (Ctrl+c)같이 시그널이 오면 멈춤
    retval = mutex_lock_interruptible(&dev->io_mutex);
    
    // 0은 mutex에서 열쇠 얻기 성공, 0 이하는 에러나 뭔가 오류가 생겼다는 의미입니다.
    if (retval < 0) {
        return retval;
    }

    // 장치가 연결 해제되었다면(disconnected)
    if (dev->disconnected) {
        retval = -ENODEV;
        goto exit;
    }

retry:
    /* * 스핀락 : 아주 짧은 시간(나노초 단위) 동안 공유 데이터를 안전하게 확인합니다.
    *  Busy-wait : 잠들지 않고 제자리에서 문이 열릴 때까지 기다립니다. (매우 빠름)
    *  _irq : 값을 확인하는 찰나의 순간에도 하드웨어 방해(인터럽트)를 받지 않기 위해 사용합니다.
    * !!주의!! : 이 안에서는 절대로 잠을 자거나(Sleep) 오래 걸리는 작업을 하면 안 됩니다!
    */
    spin_lock_irq(&dev->err_lock);
    ongoing_io = dev->ongoing_read;
    spin_unlock_irq(&dev->err_lock);

    printk("[USB_READ][usb_read] : ongoing id : %d\n",ongoing_io);

    // 데이터가 오는 중인지 확인함
    if (ongoing_io) {

        // 유저가 "기다리기 싫다"고 설정(O_NONBLOCK)했다면
        if (file->f_flags & O_NONBLOCK) {
            retval = -EAGAIN;   // 오류 반환
            goto exit;
        }

        //데이터가 다 올 때까지 프로세스를 재웁니다.
        // dev->bulk_in_wait: 이 프로세스가 잠들 상태 변수 주소
        // !dev->ongoing_read: 깨울 조건(여기서는 ongoing_read이 false가 될때 깨워짐)
        printk("[USB_READ][usb_read] : wait event for read callback!!\n");
        retval = wait_event_interruptible(dev->bulk_in_wait, (!dev->ongoing_read));
        if (retval < 0) {
            goto exit;
        }
        printk("[USB_READ][usb_read] : success to wake up by read callback!!\n");
    }

    printk("[USB_READ][usb_read] : check for errors\n");
    // 장치 에러에 관한 처리
    retval = dev->errors;
    if (retval < 0) {
        // 에러가 한번이라도 발생했다면
        dev->errors = 0;

        // 상세한 USB 에러들 중, 복구가 필요한 '-EPIPE(Stall)'는 그대로 유지하고
        // 나머지는 유저가 이해하기 쉬운 일반적인 입출력 에러('-EIO')로 변환합니다.
        retval = (retval == -EPIPE) ? retval : -EIO;

        // 에러 리포트
        goto exit;
    }
    printk("[USB_READ][usb_read] : no errors\n");
        /* 
         * 읽기에 만족할 만큼 버퍼가 찼는지 확인하고
         * 아니라면, 다시 기다립니다
        */
    printk("[USB_READ][usb_read] : check for bulk_in_filled... \n");
       if(dev->bulk_in_filled) {
            // 읽을 데이터가 있는지 확인

            /* 
            * (전체 받은 양 - 이미 유저가 읽어간 양) = 현재 줄 수 있는 남은 양
            * (남은 재고)와 (유저가 요청한 크기) 중 더 작은 값을 최종 배달 양으로 결정
            */
            printk("[USB_READ][usb_read] : bulk_in_filled : %d\n",dev->bulk_in_filled);
            size_t available = dev->bulk_in_filled - dev->bulk_in_copied;
            size_t chunk = min(available, count);

            // 유저에게 줄 데이터가 없으면 (데이터가 0)
            if (!available) {

                // 장치로 부터 데이터를 읽을 함수 호출(skel_do_read_io같은)
                retval = usb_do_read_io(dev, count);
                if (retval < 0) {
                    goto exit;
                } else {
                    goto retry;
                }
            }
            
            // 데이터가 충분하면 (0이 아니면)
            // chunk의 크기만큼 유저 공간에 copy됨
            if (copy_to_user(buffer, 
                            dev->bulk_in_buffer + dev->bulk_in_copied,
                            chunk)) {
                
                // copy to user같은경우 실패시 6을 보냄
                retval = -EFAULT;
            } else {
                printk("[USB_READ][usb_read] : success to copy to user");
                retval = chunk;
            }

            dev->bulk_in_copied += chunk;
            
            // 아직 유저가 요청한 크기의 데이터가 도착이 안됐으면
            if (available < count) 
                usb_do_read_io(dev, count - chunk);
                // << 이 부분 goto retry;하는게 낫지 않을까요?
        } else {
                // 그게 아니라면 count만큼 다시한번 요청
                usb_do_read_io(dev, count);
                if (retval < 0) {
                    goto exit;
                } else {
                    goto retry;
                }
        }

    exit:
        mutex_unlock(&dev->io_mutex);
        return retval;

}

// 패킷을 보내기 전 프로토콜을 확인하는 함수
static int check_protocol(__u8* kbuf, struct protocol_header *header, size_t actual_size) 
{

    // check if protocol start from 0x02(STX)
    /* Do NOT dereference userspace pointers in kernel context.
     * Always copy to kernel buffer first.
     */
    if ((kbuf[0] != 0x02)) {
        return -EINVAL;
    }

    printk("success check STX, kbuf[0] is %02x\n",kbuf[0]);

    // header length check
    memcpy(header,kbuf,sizeof(struct protocol_header));

    printk("success to check header length\n");

    // check LEN if different from actual_size
    // len멤버는 STX서부터 CRC전까지의 
    if (header->len != actual_size) {
        return -EMSGSIZE;
    }

    printk("success to check total length, length is : %02x\n", header->len);

    return 0;
}

static void usb_write_bulk_callback(struct urb *urb) {
    struct usb_info *dev;
    unsigned long flags;

    // usb_fill_bulk에서의 context는 디바이스임
    dev = urb->context;

    printk("[USB WRITE CALLBACK][usb write callback] : success to call write callback");

    // "URB(USB Request Block)의 연결을 끊을 때 발생하는 상태값은 하드웨어 에러로 취급하지 않겠다
    if (urb->status) {
        if (!(urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN))
			dev_err(&dev->interface->dev,
				"%s - nonzero write bulk status received: %d\n",
				__func__, urb->status);

        // read와 달리 write는 전송 후 바로 드라이버에게 제어권을 넘겨서 irq save를 굳이 status보는데 사용 안해도 됨
		spin_lock_irqsave(&dev->err_lock, flags);
		dev->errors = urb->status;
		spin_unlock_irqrestore(&dev->err_lock, flags);
        printk("[USB WRITE CALLBACK][usb write callback] : success to check urb_status");
    }

        print_hex_dump(KERN_INFO, "tx: ",
               DUMP_PREFIX_OFFSET,
               16, 1,
               urb->transfer_buffer,
               urb->actual_length,
               true);

    // 버퍼 할당 후 해제
	usb_free_coherent(urb->dev, urb->transfer_buffer_length,
			  urb->transfer_buffer, urb->transfer_dma);
    printk("[USB WRITE CALLBACK][usb write callback] : success to free coherent");
	up(&dev->limit_sem);
    printk("[USB WRITE CALLBACK][usb write callback] : success to up urb semaphore");

}

// userspace에서 write할때 호출됨
static ssize_t usb_write(struct file *file, const char *user_buffer, size_t count, loff_t *ppos) {
    struct usb_info *dev;
    int retval = 0;
    struct urb *urb = 0;
    char *buf = NULL;
    size_t writesize = min_t(size_t, count, MAX_TRANSFER);

    dev = file->private_data;

    // write할 버퍼 길이를 확인
    if (count == 0) {
        goto exit;
    }
    printk("[USB WRITE][usb_write] : success to check write_size : %ld!!\n", count);

    // USB 전송을 너무 많이 해서 시스템에 무리가 가지 않도록, 빈자리가 생길 때까지 프로세스를 재우거나(Blocking) 에러를 보내는(Non-blocking) 로직
    if (!(file->f_flags & O_NONBLOCK)) {
		if (down_interruptible(&dev->limit_sem)) {
			retval = -ERESTARTSYS;
			goto exit;
		}
	} else {
		if (down_trylock(&dev->limit_sem)) {
			retval = -EAGAIN;
			goto exit;
		}
	}

    // 에러를 보기위해서 짧게 spin_lock을 돔
    spin_lock_irq(&dev->err_lock);
	retval = dev->errors;
	if (retval < 0) {
		/* 에러가 한번이라도 왔다면 */
		dev->errors = 0;
		retval = (retval == -EPIPE) ? retval : -EIO;
	}
	spin_unlock_irq(&dev->err_lock);
	if (retval < 0)
		goto error;
    printk("[USB WRITE][usb_write] : check there is no error \n");


    /* 담을 버퍼와 urb를 생성하고, urb로 보낼 데이터를 복사 */
    urb = usb_alloc_urb(0, GFP_KERNEL);
    if (!urb) {
        retval = -ENOMEM;
        goto error;
    }
    printk("[USB WRITE][usb_write] : success to alloc urb! \n");    

    /* * [메모리 할당] CPU와 USB 하드웨어가 동시에 사용할 '특수 공유 메모리'를 만듭니다.
    * 이 함수가 성공하면, 하나의 메모리 덩어리에 대해 '이름(주소)'이 두 개 생깁니다.
    */
    buf = usb_alloc_coherent(
        dev->udev,              /* 1. 이 메모리를 보고 데이터를 읽어갈 USB 장치 */
        writesize,             /* 2. 빌리고 싶은 메모리 크기 (바이트 단위) */
        GFP_KERNEL,            /* 3. 메모리 할당 옵션 (부족하면 확보될 때까지 잠시 대기 가능) */
        &urb->transfer_dma     /* 4. [중요] 하드웨어가 이 메모리에 찾아올 때 쓸 '전용 주소'를 받아올 변수 */
    );

    /* * 결과물 설명:
    * 1. urb->transfer_buffer (리턴값): CPU(우리 코드)가 접근할 때 쓰는 주소. 
    * -> 여기에 copy_from_user()로 데이터를 채워 넣습니다.
    *
    * 2. urb->transfer_dma (4번째 인자): USB 하드웨어(컨트롤러)가 접근할 때 쓰는 주소.
    * -> 하드웨어는 똑똑하지 않아서 CPU의 가상 주소를 이해 못 하므로, 이 전용 주소가 꼭 필요합니다.
    */
   printk("[USB WRITE][usb_write] : success to alloc_coherent!\n");

    /* copy_from_user()는
    CPU가 buf가 가리키는 커널 가상주소에,
    user buffer가 가리키는 userspace 가상주소의 내용을 복사한 것입니다. */
    
    if (copy_from_user(buf, user_buffer, writesize)) {
        /* * 만약 복사에 실패했다면 (반환값이 0이 아니라면):
        * 유저가 준 주소가 엉터리거나, 접근 권한이 없는 메모리일 경우입니다.
        */
        retval = -EFAULT;  /* "잘못된 메모리 주소입니다"라는 에러 코드를 설정 */
        goto error;        /* 할당받은 메모리를 해제하는 에러 처리 구간으로 이동 */
    }


    // check if protocol is ok
    if (check_protocol((__u8*)buf, &dev->header, writesize) < 0)
        goto error;

    // 데이터 전송
	mutex_lock(&dev->io_mutex);
	if (dev->disconnected) {		/* disconnect() 가 호출되면 */
		mutex_unlock(&dev->io_mutex);
		retval = -ENODEV;
		goto error;
	}

	/* urb 초기화 */
	usb_fill_bulk_urb(urb,      // urb
            dev->udev,          // device
			usb_sndbulkpipe(dev->udev, dev->bulk_out_endpointAddr),     // 보낼 endpoint 파이프
			buf,                // 전송할 버퍼 
            writesize,          // 버퍼 크기 
            usb_write_bulk_callback,       // 전송 완료 후 콜백 함수
            dev                 // context 즉 urb가 어디 장치에 있는건지 확인
        );
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

    printk( "[USB WRITE][usb_write] : success to fill urb\n");
    printk(" buf : %s\n",buf);

    // anchor urb는 여러 개의 URB를 하나의 리스트로 묶어서 관리할 수 있게 해준다. 
	usb_anchor_urb(urb, &dev->submitted);

	/* bulk port로 urb를 보냄 */
	retval = usb_submit_urb(urb, GFP_KERNEL);

	mutex_unlock(&dev->io_mutex);

    printk( "[USB WRITE][usb_write] : success to submit urb!! \n");

	if (retval) {
		dev_err(&dev->interface->dev,
			"%s - failed submitting write urb, error %d\n",
			__func__, retval);
		goto error_unanchor;
	}

    // urb 해제
	usb_free_urb(urb);

    // 성공시 자원은 건드리지 않고 return 해야함
    goto exit;
    

error_unanchor:
	usb_unanchor_urb(urb);
/* 에러가 생겼다면 할당한 urb를 놓아주고, 빈자리를 하나 생성함 */
error:
	if (urb) {
		usb_free_coherent(dev->udev, writesize, buf, urb->transfer_dma);
		usb_free_urb(urb);
	}
	up(&dev->limit_sem);


exit:
    return retval;
}

//VFS에 등록하여서 해당 파일로 된 usb 디바이스를 읽고,쓰기를 할대 해당 fops 구조체에 등록된 콜백 함수들을 호출함
static const struct file_operations usb_fops = {
    .owner = THIS_MODULE,
    .open = usb_open,       // file이 open될때 호출
    .release = usb_release,     // file 이 close 될때 호출
    .read = usb_read,
    .write = usb_write,
};

struct usb_class_driver my_usb_class = {
    .name = "myusb%d",       // /dev/myusb0, /dev/myusb1 ... 형태로 디바이스 노드 이름 지정
    .fops = &usb_fops,        // 해당 디바이스의 VFS 접근을 위한 file_operations 구조체 연결

    /*
     리눅스,유닉스 시스템이 장치를 관리하는 핵심 방법 중 하나는 메이저 번호와 마이너 번호로 장치를 관리하는 것이다.
     * major번호는 어떤 드라이버를 사용할 지 알려주는 번호이다.
     * minor번호는 같은 드라이버 내에서 특정 장치를 구별하기 위한 번호이다.
    */
    .minor_base = 0,       // 디바이스 번호 시작 값 (여러 장치가 있을 경우 offset)
}; 


//커널에서 usb드라이버를 등록해서, 포트에 해당 usb장치가 인식되었는데도 현재 커널에 맞는 드라이버가 없으면 자동으로 insmode함
MODULE_DEVICE_TABLE(usb, usb_device_table); 


//드라이버가  커널에서 로드된 상태에서 usb가 plug되면 드라이버의 probe콜백 함수가 실행된다.
static int usb_probe(struct usb_interface *interface, const struct usb_device_id *id) {

    printk(KERN_INFO, "USB device (%04X:%04X) interface number : %04X connected\n", id->idVendor, id->idProduct,USB_INTERFACE_CLASS);

    struct usb_endpoint_descriptor *bulk_in; //엔드포인트 디스크립터 구조체 포인터로 bulk타입이 안으로 들어오는 in 선언(여기서는 첫번째 벌크 in만 사용)
    struct usb_endpoint_descriptor *bulk_out; //엔드포인트 디스크립터 구조체 포인터로 bulk타입이 안으로 들어오는 out(여기서는 첫번째 벌크 out만 사용)


    /*디바이스 구조체 할당*/
    struct usb_info *dev;
    int retval;     //함수 리턴값을 저장할 변수

    /* 사용할 디바이스에 대한 메모리 할당과 초기화를 진행함 */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

    // write할때 보낼 semaphore 갯수 초기화
    sema_init(&dev->limit_sem, WRITES_IN_FLIGHT);
    dev_info(&interface->dev, "[USB PROBE][usb_probe] : success to initalize limit_semaphore!!\n");

    // mutex 초기화
    mutex_init(&dev->io_mutex);
    dev_info(&interface->dev, "[USB PROBE][usb_probe] : success to initalize mutex!!\n");

    // 링 버퍼용 mutex 초기화
    mutex_init(&dev->buf_mutex);
    dev_info(&interface->dev, "[USB PROBE][usb_probe] : success to initalize ring buffer mutex!!\n");    

    // bulk wait queue 초기화
    init_waitqueue_head(&dev->bulk_in_wait);
    dev_info(&interface->dev, "[USB PROBE][usb_probe] : success to initalize bulk_in_wait!!\n");

    // spin_lock 초기화
    spin_lock_init(&dev->err_lock);
    dev_info(&interface->dev, "[USB PROBE][usb_probe] : success to initalize spin_lock!!\n");

    //커널 레퍼런스 초기화
    kref_init(&dev->kref);
    dev_info(&interface->dev, "[USB PROBE][usb_probe] : success to initialize kref! kref status : %d\n", kref_read(&dev->kref));        //kref_read를 통해서 커널 카운트 읽기

    //이 부분을 안할 시 disconnect 과정에서 초기화 하지 않은 쓰레기값을 참조하게 되는 오류가 생김
    init_usb_anchor(&dev->submitted);
    dev_info(&interface->dev, "[USB PROBE][usb_probe] : success to initalize anchor!!\n");    
    
    // 구조체 내부 rx_buffer 관련 head, tail 구조체 초기화
    dev->head = 0;
    dev->tail = 0;



    /*usb 디바이스, 인터페이스 구조체 포인터 할당*/
    dev->udev = usb_get_dev(interface_to_usbdev(interface)); //usb_get_dev
    
    //인터페이스에 대한 참조 카운트 증가 후 인터페이스 포인터를 얻어옴
    //사용이유 : 인터페이스 포인터를 dev 구조체나 다른 컨텍스트에 보관할 때, USB core가 disconnect 시 free 하지 못하도록 보호
    dev->interface = usb_get_intf(interface); 
    

    /*엔드포인트 정보 설정*/
    /* 여기에선 오직 첫번째 bulk_out과 bulk_in 을 사용한다.*/

    // 장치와 실제 데이터를 주고받을 통로(bulk_in, bulk_out)를 설정하기 위해,
    // 성공 시 bulk_in과 bulk_out 포인터 변수에 해당 엔드포인트 디스크립터의 주소가 저장된다.
    // usb_find_common_endpoints는 cur_altsetting, &bulk_in, &bulk_out, interrupt_in, interrupt_out에 매개변수로 이루어져있다. 
    retval = usb_find_common_endpoints(interface->cur_altsetting, &bulk_in, &bulk_out, NULL, NULL);
    
    //endpoint를 찾지 못할 시
    if (retval) {
        dev_err(&interface->dev, "Could not find both bulk-in and bulk-out endpoints\n");
    }

    //이 엔드포인트가 한 번에 처리할 수 있는 최대 패킷 크기(wMaxPacketSize)를 저장해 둔다
    dev->bulk_in_size = usb_endpoint_maxp(bulk_in);
    dev->bulk_in_endpointAddr = bulk_in->bEndpointAddress;      //read할 bulk_in 엔드포인트 주소를 얻음

    //USB 장치로부터 받은 데이터를 저장할 메모리 공간(버퍼)을 커널에 요청, GFP_KERNEL은 커널에 메모리가 없을 시 sleep한다는 플래그(설정)임
    dev->bulk_in_buffer = kmalloc(dev->bulk_in_size, GFP_KERNEL);       
    
    //만약 커널 버퍼를 할당받는데 실패할시
    if (!dev->bulk_in_buffer) {
		retval = -ENOMEM;
		goto error;
	}

    //USB 데이터 전송 구조체(URB, USB Request Block)를 위한 메모리를 할당한다.
    dev->bulk_in_urb = usb_alloc_urb(0, GFP_KERNEL);

    //urb할당에 실패할 시
    if(!dev->bulk_in_urb) {
        retval = -ENOMEM;
        goto error;
    }

    //bulk_out 엔드포인트 주소 설정(write시 보내기 위함)
    dev->bulk_out_endpointAddr = bulk_out->bEndpointAddress;

    // 이 드라이버의 **개인 데이터(dev)를 커널의 interface 구조체에 연결(link)**한다(probe 함수가 끝나면 로컬 구조체인 dev가 해제되기 때문).
    // 이를 통해 나중에 read, write, disconnect 등 다른 함수에서 usb_get_intfdata로 dev 포인터를 쉽게 다시 얻을 수 있다)
    usb_set_intfdata(interface, dev);

    //Todo VFS에 fops를 등록하기 위한 함수이다, fops class 구조체 및 fops 구현을 해야함
    //VFS에 fops를 등록 /dev/myusb0 같은 장치 노드를 생성
    // probe 함수 안에서 호출해서 해당 인터페이스와 fops(class driver)를 연결
    retval = usb_register_dev(interface, &my_usb_class);
    if (retval)
    {
        dev_err(&interface->dev, "unable to register file operations");
        goto error;
    }

    dev_info(&interface->dev, "[USB PROBE][usb_probe] : success to register fops retval %d", retval);

    /* 
    * dev_info vs printk
    * dev_info: 특정 장치(struct device *)와 연관된 로그를 남김
    *        -> dmesg나 udevadm info에서 장치 context 확인 가능
    * printk: 단순 커널 로그 출력, 장치 정보 없음
    *        -> dev가 등록되기 전/장치와 무관한 로그에 사용 
    */
    dev_info(&interface->dev, "[USB PROBE][usb_probe] : usb device now attached to myusb-%d", interface->minor);
    return retval;
    
error:
    /* 할당한 메모리 free*/
    kref_put(&dev->kref, usb_delete);
    return 0;
}



//usb 연결이 포트로부터 해제되면 호출되는 콜백 함수이다.
static void usb_disconnect(struct usb_interface *interface) {
    
	struct usb_info *dev;
    
    //드라이버 내에서 장치를 구분한 minor번호
    int minor = interface->minor;

    //probe 단계에서 set한 interface data 정보를 다시 얻어옴
    dev = usb_get_intfdata(interface);

    //VFS에 연결되어있는 fops정보들을 deregister, 즉 해제함 중요!! 커널에는 순서가 중요함 이거 없어서 여태까지 crash가 난거였음
    usb_deregister_dev(interface, &my_usb_class);
    dev_info(&interface->dev, "[USB DISCONNECTED][usb_disconnected] : Success to deregister_dev");

    //(임계영역일듯) 동시 접근을 막기 위해 mutex lock을 함
    mutex_lock(&dev->io_mutex);
    dev->disconnected = 1;  //dev 구조체에 있는 disconnected flag값을 1로 설정
    mutex_unlock(&dev->io_mutex);

    //메모리에 있는 내용을 제거하기 전에 버퍼에 내용이 남아있는지 확인
    if(dev->bulk_in_urb) {
        //read나 write중인 디바이스에 있는 urb 내용을 제거함(아직 메모리 해제 안됐음)
        usb_kill_urb(dev->bulk_in_urb);    
        dev_info(&interface->dev, "[USB DISCONNECTED][usb_disconnected] : success to kill urbs\n");
    }

    //urb를 kill 하기전에 비어있는지부터 확인
    if(!usb_anchor_empty(&dev->submitted)) {
        dev_info(&interface->dev, "[USB DISCONNECTED][usb_disconnected] : success to kill anchored_urbs\n");
        //콜백 대기중인 urb들도 제거함(장치가 제거되고 메모리가 free되었는데 해당 urb가 장치에 접근하면 메모리 오류가 나기 때문)
        usb_kill_anchored_urbs(&dev->submitted);
        dev_info(&interface->dev, "[USB DISCONNECTED][usb_disconnected] : success to kill anchored_urbs\n");
    }

    int pre_kref = kref_read(&dev->kref);  //kref로그를 위해 kref put 하기 전에 읽어옴

    //디바이스 참조 포인트를 줄임 시스템 프로그래밍에서 get은 참조 카운트를 1올리고, put은 참조 카운트를 1 내리는걸 뜻함
    //첫 번째 인자는 레퍼런스 카운트를 1 줄일 대상이고, 2번째 인자는 레퍼런스 카운트가 0일 때 호출할 함수를 뜻함
    //kref는 보통 공용으로 쓰는 자원일때 바로 free해버리면 다른 사용자가 read나 write하고 있을때 메모리 오류가 생길 수 있기 때문에
    //1 줄이고 요청한 프로세스는 계속 put을 확인함, kref가 0이 되었을때 비로소 release 즉 해제를 함
    kref_put(&dev->kref, usb_delete);

    dev_info(&interface->dev, "[USB DISCONNECTED][usb_disconnected] : success to decrease kref %d -> %d\n", pre_kref, kref_read(&dev->kref));

    dev_info(&interface->dev, "[USB DISCONNECTED][usb_disconnected] : myUsb#%d now disconnected", minor);

	dev_info(&interface->dev, "[USB DISCONNECTED][usb_disconnected] : USB device disconnected\n");
}

//usb core에 등록할 드라이버 정보 구조체이다.
static struct usb_driver my_usb_driver = {
    .name = "miniPLC-usb-driver",  //usb core에 등록할 드라이버 구조체 이름이다.
    .id_table = usb_device_table,  // 지원하는 VID/PID 매칭
    .probe = usb_probe,   //probe단계에서 호출할 콜백 함수이다.
    .disconnect = usb_disconnect,  //usb 연결이 끓어졌을 때 호출할 함수이다.
};

//insmod시 호출될 함수
static int __init usb_init(void) {
    return usb_register(&my_usb_driver);
}

//rmmod시 호출될 함수
static void __exit usb_exit(void) {
    usb_deregister(&my_usb_driver);
}


module_init(usb_init);
module_exit(usb_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("JaemoPark");
MODULE_DESCRIPTION("A custom Linux USB device driver to interface with miniPLC hardware via bulk transfer.");

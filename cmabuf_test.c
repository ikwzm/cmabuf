#include        <stdio.h>
#include        <fcntl.h>
#include        <string.h>
#include        <sys/types.h>
#include        <sys/mman.h>

void main()
{
  int            fd;
  int            i;
  unsigned char  attr[1024];
  unsigned char* buf;
  unsigned int   buf_size;
  unsigned long  phys_addr;
  unsigned long  debug_vma = 0;

  fd  = open("/sys/class/cmabuf/cmabuf0/phys_addr", O_RDONLY);
  read(fd, attr, 1024);
  sscanf(attr, "%x", &phys_addr);
  close(fd);

  fd  = open("/sys/class/cmabuf/cmabuf0/size"     , O_RDONLY);
  read(fd, attr, 1024);
  sscanf(attr, "%d", &buf_size);
  close(fd);

  fd  = open("/sys/class/cmabuf/cmabuf0/debug_vma", O_WRONLY);
  sprintf(attr, "%d", debug_vma);
  write(fd, attr, strlen(attr));
  close(fd);

  printf("phys_addr=0x%x\n", phys_addr);
  printf("size=%d\n", buf_size);

  fd  = open("/dev/cmabuf0", O_RDWR | O_SYNC);
  buf = mmap(NULL, buf_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
  for(i = 0; i < buf_size; i++) {
    buf[i] = i & 0xFF;
  }
  close(fd);
}


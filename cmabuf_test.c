#include        <stdio.h>
#include        <fcntl.h>
#include        <string.h>
#include        <time.h>
#include        <sys/types.h>
#include        <sys/mman.h>

void print_diff_time(struct timeval start_time, struct timeval end_time)
{
  struct timeval diff_time;
  if (end_time.tv_usec < start_time.tv_usec) {
    diff_time.tv_sec  = end_time.tv_sec  - start_time.tv_sec  - 1;
    diff_time.tv_usec = end_time.tv_usec - start_time.tv_usec + 1000*1000;
  } else {
    diff_time.tv_sec  = end_time.tv_sec  - start_time.tv_sec ;
    diff_time.tv_usec = end_time.tv_usec - start_time.tv_usec;
  }
  printf("time = %ld.%06ld sec\n", diff_time.tv_sec, diff_time.tv_usec);
}

int check_buf(unsigned char* buf, unsigned int size)
{
  const int m = 256;
  int n = 10;
  int i, k;
  int error_count = 0;
  while(--n > 0) {
    for(i = 0; i < size; i = i + m) {
      for(k = 0; k < m; k++)
        buf[i+k] = (k & 0xFF);
      for(k = 0; k < m; k++)
        if (buf[i+k] != (k & 0xFF)){error_count++;}
    }
  }
  return error_count;
}

void main()
{
  int            fd;
  unsigned char  attr[1024];
  unsigned char* buf;
  unsigned int   buf_size;
  unsigned long  phys_addr;
  unsigned long  debug_vma = 0;
  int            error_count;
  struct timeval start_time, end_time;

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

  fd  = open("/dev/cmabuf0", O_RDWR);
  buf = mmap(NULL, buf_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
  gettimeofday(&start_time, NULL);
  error_count = check_buf(buf, buf_size);
  gettimeofday(&end_time  , NULL);
  print_diff_time(start_time, end_time);
  close(fd);

  fd  = open("/dev/cmabuf0", O_RDWR | O_SYNC);
  buf = mmap(NULL, buf_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
  gettimeofday(&start_time, NULL);
  error_count = check_buf(buf, buf_size);
  gettimeofday(&end_time  , NULL);
  print_diff_time(start_time, end_time);
  close(fd);
}


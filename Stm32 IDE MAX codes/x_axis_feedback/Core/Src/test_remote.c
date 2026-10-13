#define REMOTE_JOYSTICK_HOST_TEST
#include "remote_joystick.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>
static void feed(const char *s, uint32_t t){ for(; *s; s++) Remote_Feed((uint8_t)*s, t); }
int test_remote(void)
{
  assert(Remote_Get(0, 2048)==2048);               /* nothing yet -> centre */
  feed("2051\n",100);  assert(Remote_Get(150,0)==2051);
  feed("0\r\n",110);   assert(Remote_Get(150,0)==0);
  feed("4095\n",120);  assert(Remote_Get(150,0)==4095);
  feed("4096\n",130);  assert(Remote_Get(150,0)==4095);   /* out of range ignored */
  feed("12345\n",131); assert(Remote_Get(150,0)==4095);   /* too long ignored */
  feed("12x4\n",132);  assert(Remote_Get(150,0)==4095);   /* garbage ignored */
  feed("\n",133);      assert(Remote_Get(150,0)==4095);   /* empty ignored */
  feed("30",140); feed("00\n",141); assert(Remote_Get(150,0)==3000); /* split across calls */
  feed("77\n",200);    assert(Remote_Get(300,2048)==77);
  assert(Remote_Get(200+250,2048)==77);                    /* exactly at timeout still ok */
  assert(Remote_Get(200+251,2048)==2048);                  /* stale -> centre (fail-safe) */
  feed("999\n",1000);  assert(Remote_Get(1010,0)==999);    /* recovers */
  puts("ALL REMOTE PARSER TESTS PASSED");
  return 0; }

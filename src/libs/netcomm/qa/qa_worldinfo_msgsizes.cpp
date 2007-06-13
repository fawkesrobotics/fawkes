
#include <netcomm/worldinfo/messages.h>

#include <stdio.h>

int
main(int argc, char **argv)
{

  printf("sizeof(worldinfo_message_header_t) = %lu\n", sizeof(worldinfo_message_header_t));
  printf("sizeof(worldinfo_header_t) = %lu\n", sizeof(worldinfo_header_t));
  printf("sizeof(worldinfo_pose_message_t) = %lu\n", sizeof(worldinfo_pose_message_t));
  printf("sizeof(worldinfo_velocity_message_t) = %lu\n", sizeof(worldinfo_velocity_message_t));
  printf("sizeof(worldinfo_relballpos_message_t) = %lu\n", sizeof(worldinfo_relballpos_message_t));
  printf("sizeof(worldinfo_relballvelo_message_t) = %lu\n", sizeof(worldinfo_relballvelo_message_t));
  printf("sizeof(worldinfo_opppose_message_t) = %lu\n", sizeof(worldinfo_opppose_message_t));
  printf("sizeof(worldinfo_fat_message_t) = %lu\n", sizeof(worldinfo_fat_message_t));

  return 0;
}


BEGIN TRANSACTION;
CREATE TABLE config (
  path      TEXT NOT NULL,
  type      TEXT NOT NULL,
  value     NOT NULL,
  comment   TEXT,
  PRIMARY KEY (path)
);
INSERT INTO "config" VALUES('/fawkes/mainapp/blackboard_size','unsigned int','2097152','Size of BlackBoard memory segment; bytes');
INSERT INTO "config" VALUES('/fawkes/mainapp/desired_loop_time','unsigned int',30000,'Desired loop time of main thread; microseconds; 0 to disable');
INSERT INTO "config" VALUES('/fawkes/mainapp/max_thread_time','unsigned int',30000,'Maximum time a thread may run per loop; microseconds; 0 to disable');
INSERT INTO "config" VALUES('/firevision/fountain/tcp_port','unsigned int',2208,'Fountain TCP Port');
INSERT INTO "config" VALUES('/worldinfo/multicast_addr','string','224.16.0.1','Multicast address to send world info messages to.');
INSERT INTO "config" VALUES('/worldinfo/udp_port','unsigned int','2806','UDP port to listen for and send world info messages to.');
INSERT INTO "config" VALUES('/worldinfo/encryption_key','string','AllemaniACsX0rz','Default encryption key for world info.');
INSERT INTO "config" VALUES('/worldinfo/encryption_iv','string','DoesAnyOneCare','Default encryption initialization vector for world info.');
INSERT INTO "config" VALUES('/worldinfo/enable_fatmsg','bool','0','Send legacy fat message?');
INSERT INTO "config" VALUES('/webview/port','unsigned int','10117','TCP port for Webview HTTP requests');
INSERT INTO "config" VALUES('/ballposlog/log_level','unsigned int','0','Log level for ballposlog example plugin; sum of any of debug=0, info=1, warn=2, error=4, none=8');
INSERT INTO "config" VALUES('/skiller/skillspace','string','test','Skill space');
INSERT INTO "config" VALUES('/skiller/watch_files','bool',1,'Watch lua files for modification and automatically reload Lua if files have been changed; true to enable');
INSERT INTO "config" VALUES('/skiller/interfaces/test/reading/navigator','string','NavigatorInterface::Navigator',NULL);
INSERT INTO "config" VALUES('/skiller/interfaces/test/reading/wm_pose','string','ObjectPositionInterface::WM Pose',NULL);
INSERT INTO "config" VALUES('/skiller/interfaces/test/reading/speechsynth','string','SpeechSynthInterface::Flite',NULL);
INSERT INTO "config" VALUES('/joystick/device_file','string','/dev/js0',NULL);
INSERT INTO "config" VALUES('/laser/interface_type','string','usb','Interface type, currently only usb');
INSERT INTO "config" VALUES('/laser/use_default','bool',0,'Use default settings from flash?');
INSERT INTO "config" VALUES('/laser/set_default','bool',0,'Store default settings in flash?');
INSERT INTO "config" VALUES('/laser/rotation_freq','unsigned int',20,'Maximum rotation frequency; Hz');
INSERT INTO "config" VALUES('/laser/max_pulse_freq','unsigned int',10800,'Max pulse frequency; Hz');
INSERT INTO "config" VALUES('/laser/angle_step','unsigned int',16,'Angle step; 1/16th degree');
INSERT INTO "config" VALUES('/laser/profile_format','unsigned int',256,'Profile format, 0x0100 only distances, 0x0400 only echoes, 0x0500 both');
INSERT INTO "config" VALUES('/laser/can_id','unsigned int',683,'CAN ID of laser');
INSERT INTO "config" VALUES('/laser/can_id_resp','unsigned int',1808,'CAN ID response');
INSERT INTO "config" VALUES('/laser/sensor_id','unsigned int',16,'Sensor ID in laser');
INSERT INTO "config" VALUES('/laser/sensor_id_resp','unsigned int',3,'Sensor ID response');
INSERT INTO "config" VALUES('/laser/btr0btr1','unsigned int',20,'Baud rate key, 0x14 for 1 MBit/s');
INSERT INTO "config" VALUES('/laser/port','unsigned int',0,'Port, 0 for default');
INSERT INTO "config" VALUES('/laser/irq','unsigned int',0,'IRQ, 0 for default');
INSERT INTO "config" VALUES('/worldmodel/confspace','string','trunk',NULL);
INSERT INTO "config" VALUES('/worldmodel/interfaces/trunk/pose/type','string','ObjectPositionInterface',NULL);
INSERT INTO "config" VALUES('/worldmodel/interfaces/trunk/pose/from_id','string','Pose',NULL);
INSERT INTO "config" VALUES('/worldmodel/interfaces/trunk/pose/to_id','string','WM Pose',NULL);
INSERT INTO "config" VALUES('/worldmodel/interfaces/trunk/pose/method','string','copy',NULL);
INSERT INTO "config" VALUES('/worldmodel/interfaces/trunk/obstacles/type','string','ObjectPositionInterface',NULL);
INSERT INTO "config" VALUES('/worldmodel/interfaces/trunk/obstacles/from_id','string','*Obstacle*',NULL);
INSERT INTO "config" VALUES('/worldmodel/interfaces/trunk/obstacles/to_id','string','WM Obstacle %u',NULL);
INSERT INTO "config" VALUES('/worldmodel/interfaces/trunk/obstacles/method','string','copy',NULL);
INSERT INTO "config" VALUES('/worldmodel/interfaces/trunk/ball/type','string','ObjectPositionInterface',NULL);
INSERT INTO "config" VALUES('/worldmodel/interfaces/trunk/ball/from_id','string','*Ball*',NULL);
INSERT INTO "config" VALUES('/worldmodel/interfaces/trunk/ball/to_id','string','WM Ball',NULL);
INSERT INTO "config" VALUES('/worldmodel/interfaces/trunk/ball/method','string','average',NULL);
INSERT INTO "config" VALUES('/worldmodel/interfaces/trunk/gamestate/type','string','GameStateInterface',NULL);
INSERT INTO "config" VALUES('/worldmodel/interfaces/trunk/gamestate/from_id','string','WI GameState',NULL);
INSERT INTO "config" VALUES('/worldmodel/interfaces/trunk/gamestate/to_id','string','WM GameState',NULL);
INSERT INTO "config" VALUES('/worldmodel/interfaces/trunk/gamestate/method','string','copy',NULL);
COMMIT;

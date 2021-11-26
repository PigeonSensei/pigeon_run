# pigeon_run

[Pigeon Motor Driver](https://github.com/PigeonSensei/pigeon_motor_driver) Tester Ros Package

### Dependency package

- [motor_driver_msgs](https://github.com/PigeonSensei/pigeon_motor_driver/tree/master/motor_driver_msgs)
- [encoder_msgs](https://github.com/PigeonSensei/pigeon_encoder_driver/tree/master/encoder_msgs)
- [FTXUI](https://github.com/ArthurSonzogni/FTXUI)

### Run

```bash
rosrun pigeon_run motor_driver_tester_node
```

### Published Topics

- **motor_command** ([motor_driver_msgs/MotorCommand](https://github.com/PigeonSensei/pigeon_motor_driver/blob/master/motor_driver_msgs/msg/MotorCommand.msg))

### Subscribed Topics

- **encoder_count** ([encoder_msgs/EncoderCount](https://github.com/PigeonSensei/pigeon_encoder_driver/blob/master/encoder_msgs/msg/EncoderCount.msg))


### Demo

![motor_driver_tester_deme](./demo/demo.gif)

### Input Key

**select Item** : W,S

**Change value** : A, S

**at Reset value** : X

**all Reset value** : Z


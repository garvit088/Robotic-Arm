o1_angle = self.link.rx_obj(obj_type=type(servo1_angle), obj_byte_size=servo1_size)
        rec_servo2_angle = self.link.rx_obj(obj_type=type(servo2_angle), obj_byte_size=servo2_size,start_pos=servo1_size)
        rec_servo3_
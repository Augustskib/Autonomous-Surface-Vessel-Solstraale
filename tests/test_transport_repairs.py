"""Regression tests of real callbacks with fake transport, no ROS/hardware."""
from datetime import datetime, timezone, time as daytime
from types import SimpleNamespace as NS
from unittest.mock import Mock
import unittest
from test_regressions import load, instance

class TransportRepairs(unittest.TestCase):
    def motor(self):
        cls = load('boat_hardware_interface/boat_hardware_interface/motor_driver.py',
                   ['MotorDriver'], serial=NS(SerialException=OSError)).MotorDriver
        obj = instance(cls)
        obj._serial_rx_buf = b''
        obj.mock = False
        obj.ser = Mock()
        obj.ser.write.side_effect = lambda payload: len(payload)
        obj._link_needs_reset = False
        obj._reset_stop_seen = False
        obj.estop_active = False
        obj.max_thrust_pct = 100
        obj._stopped_logged = False
        return obj

    def test_reconnect_requires_stop_then_reset_after_connection(self):
        obj = self.motor()
        obj.ser.write.side_effect = OSError('lost')
        obj.thrust_callback(NS(left=40, right=40))
        self.assertIsNone(obj.ser)
        self.assertTrue(obj.estop_active)
        obj.estop_callback(NS(data=True))  # Offline STOP does not rearm.
        obj.estop_callback(NS(data=False))
        obj.ser = Mock()
        obj.ser.write.side_effect = lambda payload: len(payload)
        obj.thrust_callback(NS(left=40, right=40))
        self.assertEqual(obj.ser.write.call_args.args[0], b'L:0 R:0\n')
        obj.estop_callback(NS(data=False))
        self.assertTrue(obj.estop_active)
        obj.estop_callback(NS(data=True))
        obj.estop_callback(NS(data=False))
        obj.thrust_callback(NS(left=40, right=40))
        self.assertEqual(obj.ser.write.call_args.args[0], b'L:40 R:40\n')

    def test_partial_write_latches_stop(self):
        obj = self.motor()
        obj.ser.write.side_effect = None
        obj.ser.write.return_value = 2
        obj._write_serial(20, 20)
        self.assertTrue(obj._link_needs_reset)
        self.assertIsNone(obj.ser)

    def test_read_failure_latches_stop(self):
        obj = self.motor()
        obj.ser.in_waiting = 20
        obj.ser.read.side_effect = OSError('lost')
        obj._read_serial()
        self.assertTrue(obj._link_needs_reset)
        self.assertIsNone(obj.ser)

    def gps(self, data, seconds=100):
        registry = {}
        def parse(line, check):
            assert check is True
            if line not in registry:
                raise ValueError('bad checksum or sentence')
            return registry[line]
        def fix():
            return NS(header=NS(stamp=None, frame_id=''), status=NS(status=-1))
        cls = load('boat_hardware_interface/boat_hardware_interface/gps_publisher.py',
                   ['GPSPublisher'], serial=NS(SerialException=OSError),
                   pynmea2=NS(parse=parse, ParseError=ValueError),
                   datetime=datetime, timezone=timezone,
                   NavSatFix=fix, NavSatStatus=NS(STATUS_FIX=0, STATUS_NO_FIX=-1)).GPSPublisher
        obj = instance(cls)
        # Module-level fake clock for UTC; ROS stamp deliberately matches it.
        cls.read_gps.__globals__['time'] = NS(time=lambda: seconds)
        obj.get_clock = Mock(return_value=NS(now=lambda: NS(to_msg=lambda: NS(sec=seconds,nanosec=0))))
        obj.ser = Mock(in_waiting=len(data))
        obj.ser.read.return_value = data
        obj._rx_buf = b''
        obj._discard_line = False
        obj.publisher_ = Mock()
        for ident, sec in [('old',90),('new',99),('last',100)]:
            registry['$GPGGA,'+ident] = NS(timestamp=daytime(0,sec//60,sec%60),
                gps_qual=1, latitude=float(sec), longitude=20, altitude=0)
            registry['$GPGGA,'+ident].latitude = 28 + sec/1000
        return obj, registry

    def test_backlog_publishes_only_newest_fresh_complete_fix(self):
        obj, _ = self.gps(b'$GPGGA,old\n$GPGGA,new\n$GPRMC,ignored\n$GPGGA,last\n')
        obj.read_gps()
        obj.publisher_.publish.assert_called_once()
        self.assertAlmostEqual(obj.publisher_.publish.call_args.args[0].latitude,28.1)

    def test_stale_and_bad_sentences_do_not_publish(self):
        obj, _ = self.gps(b'$GPGGA,old\n$GPGGA,bad\n')
        obj.read_gps()
        obj.publisher_.publish.assert_not_called()

    def test_unsynced_pi_clock_does_not_silence_gps(self):
        obj, registry = self.gps(b'', seconds=5000)
        published = 0
        for tick in range(6):
            sec = 100 + tick
            registry['$GPGGA,t%d' % tick] = NS(timestamp=daytime(0, sec // 60, sec % 60),
                gps_qual=1, latitude=28.0, longitude=20, altitude=0)
            type(obj).read_gps.__globals__['time'] = NS(time=lambda s=5000 + tick: s)
            data = b'$GPGGA,t%d\n' % tick
            obj.ser = Mock(in_waiting=len(data))
            obj.ser.read.return_value = data
            obj.read_gps()
            published = obj.publisher_.publish.call_count
            if tick < 4:
                self.assertEqual(published, 0)
        self.assertEqual(published, 2)

    def test_usb_drop_closes_port_instead_of_crashing(self):
        obj, _ = self.gps(b'')
        type(obj.ser).in_waiting = property(lambda self: (_ for _ in ()).throw(OSError('gone')))
        obj.read_gps()
        self.assertIsNone(obj.ser)

    def test_acquisition_age_is_preserved(self):
        obj, _ = self.gps(b'$GPGGA,new\n')
        obj.read_gps()
        self.assertEqual(obj.publisher_.publish.call_args.args[0].header.stamp.sec,99)

    def test_partial_sentence_survives_next_tick(self):
        obj, _ = self.gps(b'$GPGGA,ne')
        obj.read_gps()
        obj.publisher_.publish.assert_not_called()
        obj.ser.in_waiting = 2
        obj.ser.read.return_value = b'w\n'
        obj.read_gps()
        obj.publisher_.publish.assert_called_once()

    def test_overload_drops_queue_instead_of_replaying(self):
        obj, _ = self.gps(b'')
        obj.ser.in_waiting = 20000
        obj.read_gps()
        obj.ser.reset_input_buffer.assert_called_once()
        obj.publisher_.publish.assert_not_called()

    def test_midnight_age_wrap(self):
        obj, registry = self.gps(b'$GPGGA,new\n',seconds=86400)
        registry['$GPGGA,new'].timestamp = daytime(23,59,59)
        obj.read_gps()
        self.assertEqual(obj.publisher_.publish.call_args.args[0].header.stamp.sec,86399)


class NewSensorFaultTests(unittest.TestCase):
    def test_no_implicit_mock_when_i2c_dependency_missing(self):
        cls = load('boat_hardware_interface/boat_hardware_interface/imu_publisher.py',
                   ['ImuPublisher'], SMBus=None, MockBackend=lambda: 'mock').ImuPublisher
        obj=instance(cls)
        with self.assertRaises(RuntimeError): obj._build_backend('mpu6050',1,0,False)
        self.assertEqual(obj._build_backend('mpu6050',1,0,True),'mock')

    def test_motor_status_has_receive_time_and_raw_current_units(self):
        cls=load('boat_hardware_interface/boat_hardware_interface/motor_driver.py',
                 ['MotorDriver'], String=lambda **kw:NS(**kw)).MotorDriver
        obj=instance(cls); obj.motor_status_pub=Mock()
        obj._handle_serial_line(b'STATUS L_IS:42 R_IS:52 WDT:0 L:10 R:-20 TX_FREE:64')
        import json
        status=json.loads(obj.motor_status_pub.publish.call_args.args[0].data)
        self.assertEqual(status['reported_right_pct'],-20)
        self.assertEqual(status['time_basis'],'host_receive')
        self.assertEqual(status['current_units'],'adc_counts_not_amps')
        obj.motor_status_pub.reset_mock()
        obj._handle_serial_line(b'STATUS L_IS:42 R_IS:52 WDT:0 L:999 R:0')
        obj.motor_status_pub.publish.assert_not_called()

if __name__ == '__main__':
    unittest.main()

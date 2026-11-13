"""Explicit launch/service isolation test; runs no velocity publishers."""
from pathlib import Path
import signal
import subprocess
import tempfile
import rclpy
from rclpy.node import Node
from dock_detector_interfaces.srv import SelectDock


def test_catalogue_launches_in_two_robot_namespaces():
    catalogue=Path(__file__).parents[1]/'config/seggy_docks.yaml'
    processes=[];logs=[]
    rclpy.init();peer=Node('namespaced_selection_test')
    try:
        for namespace in ('robot_a','robot_b'):
            log=tempfile.TemporaryFile(mode='w+')
            logs.append(log)
            processes.append(subprocess.Popen(['ros2','launch','dock_detector','detector.launch.py',
                f'params_file:={catalogue}',f'namespace:={namespace}','use_sim_time:=false'],
                stdout=log,stderr=subprocess.STDOUT))
        clients=[peer.create_client(SelectDock,f'/{ns}/dock_detector/select_dock')
                 for ns in ('robot_a','robot_b')]
        for client in clients:assert client.wait_for_service(timeout_sec=10.)
        def request(index,dock,token,activate=True):
            future=clients[index].call_async(SelectDock.Request(activate=activate,dock_id=dock,acquisition_id=token))
            rclpy.spin_until_future_complete(peer,future,timeout_sec=2.)
            assert future.done();return future.result()
        assert request(0,'sim_trapezoid_bay','a').success
        assert request(1,'sim_v_bay','b').success
        assert not request(0,'sim_v_bay','b',False).success
        assert request(1,'sim_v_bay','b',False).success
        assert request(0,'sim_trapezoid_bay','a',False).success
    finally:
        for process in processes:process.send_signal(signal.SIGINT)
        for process in processes:
            try:process.wait(timeout=5.)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        for log in logs:
            log.seek(0);print(log.read());log.close()
        peer.destroy_node();rclpy.shutdown()

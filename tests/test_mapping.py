from pathlib import Path

import pytest

from web_server import mapping_controller


class DummyProcess:
    def __init__(self, cmd):
        self.cmd = cmd
        self.terminated = False
        self.wait_called = False
        self.pid = hash(tuple(cmd)) & 0xFFFF

    def poll(self):
        return None

    def terminate(self):
        self.terminated = True

    def wait(self, timeout=None):
        self.wait_called = True

    def kill(self):
        self.terminated = True


@pytest.fixture
def controller(monkeypatch, tmp_path):
    procs = []

    def fake_popen(cmd, cwd=None, stdout=None, stderr=None):
        proc = DummyProcess(cmd)
        procs.append(proc)
        return proc

    monkeypatch.setattr(mapping_controller.subprocess, "Popen", fake_popen)

    invoked_commands = []

    from types import SimpleNamespace

    def fake_run(cmd, check=False, capture_output=False, text=False):
        invoked_commands.append(cmd)
        return SimpleNamespace(returncode=0, stdout="OK", stderr="")

    monkeypatch.setattr(mapping_controller.subprocess, "run", fake_run)

    controller = mapping_controller.MappingController(
        launch_sequence=[["echo", "node1"], ["echo", "node2"]],
        map_save_script=["bash", "map_save.sh"],
        workdir=tmp_path,
    )
    controller._dummy_processes = procs
    controller._invoked_commands = invoked_commands
    controller._invoked_commands = invoked_commands
    return controller


def test_start_spawns_processes(controller):
    result = controller.start(map_name="test_map")
    assert result["status"] == "RUNNING"
    assert len(controller._dummy_processes) == 2


def test_stop_terminates_processes(controller):
    controller.start(map_name="test_map")
    controller.stop()
    assert all(proc.terminated for proc in controller._dummy_processes)


def test_stop_triggers_map_save(controller):
    controller.start(map_name="mission")
    controller.stop(save=True)
    assert any("map_save.sh" in " ".join(cmd) for cmd in controller._invoked_commands)


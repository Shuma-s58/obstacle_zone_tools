"""Exercise the real RPC synchronizer with service test doubles, without ROS."""
import ast
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS
import time


class Request:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class FakeParameter:
    def __init__(self, name, value):
        self.name, self.value = name, value

    def to_parameter_msg(self):
        return self


class Client:
    def __init__(self):
        self.calls = []
        self.ready = True

    def service_is_ready(self):
        return self.ready

    def call_async(self, req):
        f = Future()
        self.calls.append((req, f))
        return f


class FakeNode:
    def __init__(self):
        self.clients = []

    def create_client(self, *_):
        c = Client()
        self.clients.append(c)
        return c

    def get_logger(self):
        return NS(info=lambda *_: None, warning=lambda *_: None)


def make_target():
    source = Path(__file__).parents[1] / 'obstacle_zone_tools' / 'node.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'LayerTarget')
    env = {'time': time, 'GetParameters': NS(Request=Request),
           'SetParameters': NS(Request=Request), 'Parameter': FakeParameter,
           'ParameterType': NS(PARAMETER_BOOL=1)}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), env)
    node = FakeNode()
    return env['LayerTarget'](node, '/local_costmap/local_costmap', 'obstacle_layer.enabled')


def read_result(enabled):
    return NS(values=[NS(type=1, bool_value=enabled)])


def test_confirmed_transition_and_exit_during_pending_entry():
    t = make_target()
    t.step(False)
    t.get.calls[-1][1].set_result(read_result(True))
    t.step(False)
    assert len(t.set.calls) == 1
    assert t.set.calls[-1][0].parameters[0].value is False
    # Robot exits while disable is still in flight: do not send an overlapping write.
    t.step(True)
    assert len(t.set.calls) == 1
    t.set.calls[-1][1].set_result(NS(results=[NS(successful=True)]))
    t.step(True)
    assert len(t.set.calls) == 2
    assert t.set.calls[-1][0].parameters[0].value is True
    t.set.calls[-1][1].set_result(NS(results=[NS(successful=True)]))
    t.step(True)
    assert t.confirmed(True)


def test_rejected_parameter_does_not_claim_success():
    t = make_target()
    t.step(False)
    t.get.calls[-1][1].set_result(read_result(True))
    t.step(False)
    t.set.calls[-1][1].set_result(NS(results=[NS(successful=False, reason='not allowed')]))
    t.step(False)
    assert t.actual is None
    assert not t.confirmed(False)
    assert t.error == 'not allowed'


def test_external_change_is_detected_by_polling():
    t = make_target()
    t.step(True)
    t.get.calls[-1][1].set_result(read_result(True))
    t.step(True)
    assert t.confirmed(True)
    t.next_poll = 0
    t.step(True)
    t.get.calls[-1][1].set_result(read_result(False))
    t.step(True)
    assert t.set.calls[-1][0].parameters[0].value is True


def test_missing_parameter_never_written():
    t = make_target()
    t.step(False)
    t.get.calls[-1][1].set_result(NS(values=[NS(type=0)]))
    t.step(False)
    assert not t.set.calls
    assert t.actual is None
    assert 'missing' in t.error


def load_tests(loader, tests, pattern):
    import unittest
    return unittest.TestSuite(unittest.FunctionTestCase(fn) for name, fn in globals().items()
                              if name.startswith('test_') and callable(fn))

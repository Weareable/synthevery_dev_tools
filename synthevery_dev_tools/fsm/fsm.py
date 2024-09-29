from blinker import Signal
from abc import ABC, abstractmethod
from typing import Callable
from datetime import datetime, timedelta

class StateInterface(ABC):
    @abstractmethod
    def enter(self) -> None:
        pass

    @abstractmethod
    def update(self) -> None:
        pass

    @abstractmethod
    def exit(self) -> None:
        pass

class FSMInterface(ABC):
    STATE_NONE: int = 0xFFFF

    @abstractmethod
    def update(self) -> None:
        pass

    @abstractmethod
    def transition_to(self, state: int) -> None:
        pass


class StateSignals(ABC):
    @abstractmethod
    def on_enter(self) -> Signal:
        pass

    @abstractmethod
    def on_update(self) -> Signal:
        pass

    @abstractmethod
    def on_exit(self) -> Signal:
        pass


class TransitionInterface(ABC):
    @abstractmethod
    def check(self) -> bool:
        pass
    
class FSM(FSMInterface):
    class TransitionInfo:
        def __init__(self, transition: TransitionInterface, target_state: int) -> None:
            self.transition = transition
            self.target_state = target_state

    def __init__(self) -> None:
        self.states = []
        self.transitions = []
        self.current_state = FSMInterface.STATE_NONE

    def update(self) -> None:
        if self.check_state_id(self.current_state):
            self.states[self.current_state].update()

            for transition_info in self.transitions[self.current_state]:
                if transition_info.transition.check():
                    self.transition_to(transition_info.target_state)
                    break

    def check_state_id(self, state: int) -> bool:
        result = state < len(self.states) and self.states[state] is not None
        return result

    def get_current_state(self) -> StateInterface:
        return self.states[self.current_state]

    def get_current_state_id(self) -> int:
        return self.current_state

    def get_state_by_id(self, state: int) -> StateInterface | None:
        if self.check_state_id(state):
            return self.states[state]
        return None

    def get_state_id(self, state: StateInterface) -> int:
        if state in self.states:
            return self.states.index(state)
        return FSMInterface.STATE_NONE

    def add_state(self, state: StateInterface) -> int:
        self.states.append(state)
        self.transitions.append([])
        return len(self.states) - 1

    def add_transition(self, from_state: int | StateInterface, to_state: int | StateInterface, transition: TransitionInterface) -> bool:
        if isinstance(from_state, StateInterface):
            from_state = self.get_state_id(from_state)
        if isinstance(to_state, StateInterface):
            to_state = self.get_state_id(to_state)

        if self.check_state_id(from_state) and self.check_state_id(to_state):
            self.transitions[from_state].append(FSM.TransitionInfo(transition, to_state))
            return True

        return False

    def transition_to(self, state: int | StateInterface) -> None:
        if isinstance(state, StateInterface):
            state = self.get_state_id(state)

        if self.check_state_id(self.current_state):
            self.states[self.current_state].exit()

        if self.check_state_id(state):
            self.current_state = state
            self.states[self.current_state].enter()


class State(StateInterface, StateSignals):
    def __init__(self) -> None:
        self.on_enter_signal = Signal()
        self.on_update_signal = Signal()
        self.on_exit_signal = Signal()

    def enter(self) -> None:
        self.on_enter_signal.send()

    def update(self) -> None:
        self.on_update_signal.send()

    def exit(self) -> None:
        self.on_exit_signal.send()

    def on_enter(self) -> Signal:
        return self.on_enter_signal

    def on_update(self) -> Signal:
        return self.on_update_signal

    def on_exit(self) -> Signal:
        return self.on_exit_signal


class FSMTimer:
    def __init__(self, duration_ms: float) -> None:
        self.duration_ms = duration_ms
        self.start_time = datetime.now()
        self.running = False
        self.on_timeout_signal = Signal()

    def start(self) -> None:
        self.start_time = datetime.now()
        self.running = True

    def pause(self) -> None:
        self.running = False

    def resume(self) -> None:
        self.start_time = datetime.now() - (self.start_time - datetime.now())
        self.running = True

    def reset(self) -> None:
        self.start_time = datetime.now()
        self.running = False

    def update(self) -> None:
        if self.running and (datetime.now() - self.start_time > timedelta(milliseconds=self.duration_ms)):
            self.running = False
            self.on_timeout_signal.send()

    def get_elapsed_ms(self) -> float:
        if self.running:
            return (datetime.now() - self.start_time).total_seconds() * 1000
        return 0

    def bind(self, state: StateSignals) -> None:
        state.on_enter().connect(lambda sender: self.start(), weak=False)
        state.on_update().connect(lambda sender: self.update(), weak=False)

    def on_timeout(self) -> Signal:
        return self.on_timeout_signal


class FSMStopwatch:
    def __init__(self) -> None:
        self.start_time = datetime.now()
        self.running = False

    def start(self) -> None:
        self.start_time = datetime.now()
        self.running = True

    def pause(self) -> None:
        self.running = False

    def resume(self) -> None:
        self.start_time = datetime.now() - (self.start_time - datetime.now())
        self.running = True

    def reset(self) -> None:
        self.start_time = datetime.now()
        self.running = False

    def get_elapsed_ms(self) -> float:
        if self.running:
            return (datetime.now() - self.start_time).total_seconds() * 1000
        return 0

    def bind(self, state: StateSignals) -> None:
        state.on_enter().connect(lambda sender: self.start(), weak=False)


class Transition(TransitionInterface):
    def __init__(self, check_condition: Callable[[], bool]) -> None:
        self.check_condition = check_condition
        self.on_transition_signal = Signal()

    def check(self) -> bool:
        if self.check_condition():
            self.on_transition_signal.send()
            return True
        return False

    def on_transition(self) -> Signal:
        return self.on_transition_signal
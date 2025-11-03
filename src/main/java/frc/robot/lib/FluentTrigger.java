package frc.robot.lib;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FluentTrigger {
    private Command activeCommand;
    private Command defaultCommand;
    private ArrayList<Integer> activeStateQueue;
    private ArrayList<CommandBind> triggerCommandBindList;

    public FluentTrigger() {
        triggerCommandBindList = new ArrayList<CommandBind>();
        activeStateQueue = new ArrayList<Integer>();
    }

    private class CommandBind {
        Trigger trigger;
        Command command;

        private CommandBind(Trigger trigger, Command command) {
            this.trigger = trigger;
            this.command = command;
        }
    }

    public FluentTrigger setDefault(Command defaultCommand) {
        this.activeCommand = defaultCommand;
        this.defaultCommand = defaultCommand;
        updateState();
        return this;
    }

    public FluentTrigger bind(Trigger trigger, Command command) {
        CommandBind triggerCommandBind = new CommandBind(trigger, command);
        int state = triggerCommandBindList.size();
        triggerCommandBind.trigger.onTrue(new InstantCommand(() -> addQueue(state)));
        triggerCommandBind.trigger.onFalse(new InstantCommand(() -> removeQueue(state)));
        triggerCommandBindList.add(triggerCommandBind);
        return this;
    }

    private void addQueue(int state) {
        if (activeStateQueue.indexOf(state) >= 0)
            return;
        activeStateQueue.add(state);
        updateState();
    }

    private void removeQueue(int state) {
        int index = activeStateQueue.indexOf(state);
        if (index < 0)
            return;
        activeStateQueue.remove(index);
        updateState();
    }

    private void updateState() {
        if (activeStateQueue.size() == 0) {
            if (activeCommand != null && activeCommand.isScheduled() == true) {
                activeCommand.cancel();
            }
            if (defaultCommand != null && defaultCommand.isScheduled() == false) {
                defaultCommand.schedule();
            }
            return;
        }

        int activeState = activeStateQueue.get(activeStateQueue.size() - 1);
        activeCommand = triggerCommandBindList.get(activeState).command;
        if (activeCommand.isScheduled() == false) {
            activeCommand.schedule();
        }
    }
}
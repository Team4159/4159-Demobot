package frc.robot.lib;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Orchestrator extends Command {

    private enum CommandBlockStatus {
        CONTINUE, JUMP, YIELD, EXIT
    }

    @FunctionalInterface
    private interface CommandBlockCallback {
        CommandBlockStatus run();
    }

    private class CommandBlock {
        private CommandBlockCallback callback;

        public CommandBlock(CommandBlockCallback callback) {
            this.callback = callback;
        }

        public CommandBlockStatus run() {
            return callback.run();
        }
    }

    private final ArrayList<CommandBlock> blocks = new ArrayList<>();

    private final ArrayList<Command> stacks = new ArrayList<>();
    private final ArrayList<Command> untrackedStacks = new ArrayList<>();
    private final Map<String, ArrayList<Command>> trackedStacks = new HashMap<>();

    private final Map<String, Integer> labels = new HashMap<>();

    private Consumer<Boolean> exitCallback;

    private int runIndex;
    private int lastRunIndex;
    private boolean runFinished;
    private double lastBlockInitializedSeconds;

    public Orchestrator() {

    }

    @Override
    public void initialize() {
        runIndex = 0;
        lastRunIndex = 0;
        runFinished = false;
        stacks.clear();
        untrackedStacks.clear();
        trackedStacks.clear();
    }

    @Override
    public void execute() {
        if (runFinished)
            return;

        boolean stacksRunFinished = true;
        for (Command stack : stacks) {
            if (stack.isFinished() && stack instanceof Orchestrator) {
                stack.initialize();
            }
            stack.execute();
            if (!stack.isFinished()) {
                stacksRunFinished = false;
            }
        }

        while (runIndex < blocks.size()) {
            if (lastRunIndex != runIndex) {
                lastBlockInitializedSeconds = getTime();
            }
            lastRunIndex = runIndex;
            CommandBlock commandBlock = blocks.get(runIndex);
            CommandBlockStatus status = commandBlock.run();
            if (status == CommandBlockStatus.CONTINUE) {
                runIndex++;
            } else if (status == CommandBlockStatus.JUMP) {
                // do nothing
            } else if (status == CommandBlockStatus.YIELD) {
                break;
            } else if (status == CommandBlockStatus.EXIT) {
                runFinished = true;
                break;
            } else {
                throw new IllegalStateException("Invalid CommandBlockStatus");
            }
        }

        if (runIndex == blocks.size() && stacksRunFinished) {
            runFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return runFinished;
    }

    @Override
    public void end(boolean interrupted) {
        if (exitCallback != null) {
            exitCallback.accept(interrupted);
        }
        for (Command stack : stacks) {
            stack.cancel();
        }
        stacks.clear();
        untrackedStacks.clear();
        trackedStacks.clear();
        runFinished = true;
    }

    public Orchestrator require(SubsystemBase subsystem) {
        super.addRequirements(subsystem);
        return this;
    }

    public Orchestrator exit() {
        addBlock(() -> CommandBlockStatus.EXIT);
        return this;
    }

    public Orchestrator exitif(BooleanSupplier condition) {
        addBlock(() -> {
            if (condition.getAsBoolean()) {
                return CommandBlockStatus.EXIT;
            }
            return CommandBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator onexit(Consumer<Boolean> callback) {
        exitCallback = callback;
        return this;
    }

    public Orchestrator label(String name) {
        if (labels.containsKey(name)) {
            throw new IllegalArgumentException("Label already exists: " + name);
        }
        addBlock(() -> CommandBlockStatus.CONTINUE);
        addLabel(name, blocks.size() - 1);
        return this;
    }

    public Orchestrator jump(String label) {
        addBlock(() -> {
            runIndex = labels.get(label);
            return CommandBlockStatus.JUMP;
        });
        return this;
    }

    public Orchestrator jumpif(BooleanSupplier condition, String trueLabel, String falseLabel) {
        addBlock(() -> {
            if (condition.getAsBoolean()) {
                if (trueLabel != null) {
                    runIndex = labels.get(trueLabel);
                }
            } else {
                if (falseLabel != null) {
                    runIndex = labels.get(falseLabel);
                }
            }
            return CommandBlockStatus.JUMP;
        });
        return this;
    }

    public Orchestrator jumpif(BooleanSupplier condition, String trueLabel) {
        return jumpif(condition, trueLabel, null);
    }

    public Orchestrator run(Runnable callback) {
        addBlock(() -> {
            callback.run();
            return CommandBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator runif(BooleanSupplier condition, Runnable trueCallback, Runnable falseCallback) {
        addBlock(() -> {
            if (condition.getAsBoolean()) {
                if (trueCallback != null) {
                    trueCallback.run();
                }
            } else {
                if (falseCallback != null) {
                    falseCallback.run();
                }
            }
            return CommandBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator runif(BooleanSupplier condition, Runnable trueCallback) {
        return runif(condition, trueCallback, null);
    }

    public Orchestrator repeat(String group, Runnable callback) {
        addBlock(() -> {
            repeatRun(group, callback);
            return CommandBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator repeat(Runnable callback) {
        return repeat(null, callback);
    }

    public Orchestrator repeatif(BooleanSupplier condition, String trueGroup, Runnable trueCallback, String falseGroup,
            Runnable falseCallback) {
        addBlock(() -> {
            if (condition.getAsBoolean()) {
                if (trueCallback != null) {
                    repeatRun(trueGroup, trueCallback);
                }
            } else {
                if (falseCallback != null) {
                    repeatRun(falseGroup, falseCallback);
                }
            }
            return CommandBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator repeatif(BooleanSupplier condition, String trueGroup, Runnable trueCallback) {
        return repeatif(condition, trueGroup, trueCallback, null, null);
    }

    public Orchestrator repeatif(BooleanSupplier condition, Runnable trueCallback) {
        return repeatif(condition, null, trueCallback, null, null);
    }

    public Orchestrator repeatif(BooleanSupplier condition, Runnable trueCallback, Runnable falseCallback) {
        return repeatif(condition, null, trueCallback, null, falseCallback);
    }

    public Orchestrator fastrepeat(String group, Runnable callback) {
        addBlock(() -> {
            fastrepeatRun(group, callback);
            return CommandBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator fastrepeat(Runnable callback) {
        return fastrepeat(null, callback);
    }

    public Orchestrator fastrepeatif(BooleanSupplier condition, String trueGroup, Runnable trueCallback,
            String falseGroup, Runnable falseCallback) {
        addBlock(() -> {
            if (condition.getAsBoolean()) {
                if (trueCallback != null) {
                    fastrepeatRun(trueGroup, trueCallback);
                }
            } else {
                if (falseCallback != null) {
                    fastrepeatRun(falseGroup, falseCallback);
                }
            }
            return CommandBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator fastrepeatif(BooleanSupplier condition, String trueGroup, Runnable trueCallback) {
        return fastrepeatif(condition, trueGroup, trueCallback, null, null);
    }

    public Orchestrator fastrepeatif(BooleanSupplier condition, Runnable trueCallback) {
        return fastrepeatif(condition, null, trueCallback, null, null);
    }

    public Orchestrator fastrepeatif(BooleanSupplier condition, Runnable trueCallback, Runnable falseCallback) {
        return fastrepeatif(condition, null, trueCallback, null, falseCallback);
    }

    public Orchestrator command(String group, Command command) {
        addBlock(() -> {
            commandRun(group, command);
            return CommandBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator command(Command command) {
        return command(null, command);
    }

    public Orchestrator commandif(BooleanSupplier condition, String trueGroup, Command trueCommand, String falseGroup,
            Command falseCommand) {
        addBlock(() -> {
            if (condition.getAsBoolean()) {
                if (trueCommand != null) {
                    commandRun(trueGroup, trueCommand);
                }
            } else {
                if (falseCommand != null) {
                    commandRun(falseGroup, falseCommand);
                }
            }
            return CommandBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator commandif(BooleanSupplier condition, String group, Command command) {
        return commandif(condition, group, command, null, null);
    }

    public Orchestrator commandif(BooleanSupplier condition, Command command) {
        return commandif(condition, null, command, null, null);
    }

    public Orchestrator commandif(BooleanSupplier condition, Command trueCommand, Command falseCommand) {
        return commandif(condition, null, trueCommand, null, falseCommand);
    }

    public Orchestrator yield(double timeoutSeconds, BooleanSupplier condition) {
        addBlock(() -> {
            if (!condition.getAsBoolean() && getTime() - lastBlockInitializedSeconds <= timeoutSeconds) {
                return CommandBlockStatus.YIELD;
            }
            return CommandBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator yield(double seconds) {
        addBlock(() -> {
            if (getTime() - lastBlockInitializedSeconds <= seconds) {
                return CommandBlockStatus.YIELD;
            }
            return CommandBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator yield(BooleanSupplier condition) {
        addBlock(() -> {
            if (!condition.getAsBoolean()) {
                return CommandBlockStatus.YIELD;
            }
            return CommandBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator cancelStackGroup(String group) {
        addBlock(() -> {
            if (!trackedStacks.containsKey(group)) {
                // throw new IllegalAccessError("Unable to cancel stack group: " + group + "
                // because it does not exist");
                return CommandBlockStatus.CONTINUE;
            }
            ArrayList<Command> stackGroup = trackedStacks.get(group);
            for (Command stack : stackGroup) {
                stack.cancel();
                stacks.remove(stack);
            }
            trackedStacks.remove(group);
            return CommandBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator cancelAllStacks() {
        addBlock(() -> {
            for (Command stack : stacks) {
                stack.cancel();
            }
            stacks.clear();
            untrackedStacks.clear();
            trackedStacks.clear();
            return CommandBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator cancelAllUntrackedStacks() {
        addBlock(() -> {
            for (Command stack : untrackedStacks) {
                stack.cancel();
                stacks.remove(stack);
            }
            untrackedStacks.clear();
            return CommandBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator cancelAllTrackedStacks() {
        addBlock(() -> {
            trackedStacks.forEach((group, stackGroup) -> {
                for (Command stack : stackGroup) {
                    stack.cancel();
                    stacks.remove(stack);
                }
            });
            trackedStacks.clear();
            return CommandBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator print(String text) {
        addBlock(() -> {
            System.out.println(text);
            return CommandBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator print(Supplier<String> supplier) {
        addBlock(() -> {
            System.out.println(supplier.get());
            return CommandBlockStatus.CONTINUE;
        });
        return this;
    }

    private void addBlock(CommandBlockCallback callback) {
        blocks.add(new CommandBlock(callback));
    }

    private Command addStack(String group, Command stack) {
        stack.initialize();
        stacks.add(stack);
        if (group == null) {
            untrackedStacks.add(stack);
        } else {
            if (!trackedStacks.containsKey(group)) {
                trackedStacks.put(group, new ArrayList<Command>());
            }
            ArrayList<Command> stackGroup = trackedStacks.get(group);
            stackGroup.add(stack);
        }
        return stack;
    }

    private void repeatRun(String group, Runnable callback) {
        Orchestrator stack = (Orchestrator) addStack(group, new Orchestrator());
        stack.run(callback::run);
    }

    private void fastrepeatRun(String group, Runnable callback) {
        Orchestrator stack = (Orchestrator) addStack(group, new Orchestrator());
        callback.run();
        stack.run(callback::run);
    }

    private void commandRun(String group, Command command) {
        addStack(group, command);
        command.initialize();
    }

    private void addLabel(String name, int destination) {
        labels.put(name, destination);
    }

    private double getTime() {
        return Timer.getFPGATimestamp();
    }
}

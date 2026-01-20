package frc.robot.lib;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Orchestrator extends Command {

    private enum RunBlockStatus {
        CONTINUE, JUMP, YIELD, EXIT
    }

    @FunctionalInterface
    private interface RunBlockCallback {
        RunBlockStatus run();
    }

    private record RunBlock(RunBlockCallback callback) {
    }

    private record RunStack(Command command, boolean repeating) {
    }

    private final ArrayList<RunBlock> blocks = new ArrayList<>();

    private final ArrayList<RunStack> stacks = new ArrayList<>();
    private final ArrayList<RunStack> untrackedStacks = new ArrayList<>();
    private final Map<String, ArrayList<RunStack>> trackedStacks = new HashMap<>();

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
        boolean stacksRunFinished = true;
        for (RunStack stack : stacks) {
            Command command = stack.command;
            if (stack.repeating) {
                if (command.isFinished()) {
                    command.initialize();
                }
                command.execute();
            } else {
                if (!command.isFinished()) {
                    command.execute();
                    stacksRunFinished = false;
                }
            }
        }

        while (runIndex < blocks.size()) {
            if (lastRunIndex != runIndex) {
                lastBlockInitializedSeconds = getTime();
            }
            lastRunIndex = runIndex;
            RunBlock commandBlock = blocks.get(runIndex);
            RunBlockStatus status = commandBlock.callback.run();
            if (status == RunBlockStatus.CONTINUE) {
                runIndex++;
            } else if (status == RunBlockStatus.JUMP) {
                // do nothing
            } else if (status == RunBlockStatus.YIELD) {
                break;
            } else if (status == RunBlockStatus.EXIT) {
                runFinished = true;
                break;
            } else {
                throw new IllegalStateException("Invalid CommandBlockStatus");
            }
        }

        for (RunStack stack : stacks) {
            if (!stack.command.isFinished()) {
                stacksRunFinished = false;
                break;
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
        for (RunStack stack : stacks) {
            if (stack.repeating) {
                stack.command.end(true);
            } else {
                cancelStack(stack);
            }
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
        addBlock(() -> RunBlockStatus.EXIT);
        return this;
    }

    public Orchestrator exitif(BooleanSupplier condition) {
        addBlock(() -> {
            if (condition.getAsBoolean()) {
                return RunBlockStatus.EXIT;
            }
            return RunBlockStatus.CONTINUE;
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
        addBlock(() -> RunBlockStatus.CONTINUE);
        addLabel(name, blocks.size() - 1);
        return this;
    }

    public Orchestrator jump(String label) {
        addBlock(() -> {
            runIndex = labels.get(label);
            return RunBlockStatus.JUMP;
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
            return RunBlockStatus.JUMP;
        });
        return this;
    }

    public Orchestrator jumpif(BooleanSupplier condition, String trueLabel) {
        return jumpif(condition, trueLabel, null);
    }

    public Orchestrator run(Runnable callback) {
        addBlock(() -> {
            callback.run();
            return RunBlockStatus.CONTINUE;
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
            return RunBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator runif(BooleanSupplier condition, Runnable trueCallback) {
        return runif(condition, trueCallback, null);
    }

    public Orchestrator repeat(String group, Runnable callback) {
        addBlock(() -> {
            runRepeatBlock(group, callback);
            return RunBlockStatus.CONTINUE;
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
                    runRepeatBlock(trueGroup, trueCallback);
                }
            } else {
                if (falseCallback != null) {
                    runRepeatBlock(falseGroup, falseCallback);
                }
            }
            return RunBlockStatus.CONTINUE;
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

    public Orchestrator command(String group, Command command) {
        addBlock(() -> {
            runCommandBlock(group, command);
            return RunBlockStatus.CONTINUE;
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
                    runCommandBlock(trueGroup, trueCommand);
                }
            } else {
                if (falseCommand != null) {
                    runCommandBlock(falseGroup, falseCommand);
                }
            }
            return RunBlockStatus.CONTINUE;
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
                return RunBlockStatus.YIELD;
            }
            return RunBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator yield(double timeoutSeconds) {
        addBlock(() -> {
            if (getTime() - lastBlockInitializedSeconds <= timeoutSeconds) {
                return RunBlockStatus.YIELD;
            }
            return RunBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator yield(BooleanSupplier condition) {
        addBlock(() -> {
            if (!condition.getAsBoolean()) {
                return RunBlockStatus.YIELD;
            }
            return RunBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator cancelStackGroup(String group) {
        addBlock(() -> {
            if (!trackedStacks.containsKey(group)) {
                // throw new IllegalAccessError("Unable to cancel stack group: " + group + "
                // because it does not exist");
                return RunBlockStatus.CONTINUE;
            }
            var stackGroup = trackedStacks.get(group);
            for (RunStack stack : stackGroup) {
                cancelStack(stack);
                stacks.remove(stack);
            }
            trackedStacks.remove(group);
            return RunBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator cancelAllStacks() {
        addBlock(() -> {
            for (RunStack stack : stacks) {
                cancelStack(stack);
            }
            stacks.clear();
            untrackedStacks.clear();
            trackedStacks.clear();
            return RunBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator cancelAllUntrackedStacks() {
        addBlock(() -> {
            for (RunStack stack : untrackedStacks) {
                cancelStack(stack);
                stacks.remove(stack);
            }
            untrackedStacks.clear();
            return RunBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator cancelAllTrackedStacks() {
        addBlock(() -> {
            trackedStacks.forEach((group, stackGroup) -> {
                for (RunStack stack : stackGroup) {
                    cancelStack(stack);
                    stacks.remove(stack);
                }
            });
            trackedStacks.clear();
            return RunBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator print(String text) {
        addBlock(() -> {
            System.out.println(text);
            return RunBlockStatus.CONTINUE;
        });
        return this;
    }

    public Orchestrator print(Supplier<String> supplier) {
        addBlock(() -> {
            System.out.println(supplier.get());
            return RunBlockStatus.CONTINUE;
        });
        return this;
    }

    private void addBlock(RunBlockCallback callback) {
        blocks.add(new RunBlock(callback));
    }

    private RunStack addStack(String group, Command command, boolean repeating) {
        command.initialize();
        RunStack stack = new RunStack(command, repeating);
        stacks.add(stack);
        if (group == null) {
            untrackedStacks.add(stack);
        } else {
            if (!trackedStacks.containsKey(group)) {
                trackedStacks.put(group, new ArrayList<>());
            }
            var stackGroup = trackedStacks.get(group);
            stackGroup.add(stack);
        }
        return stack;
    }

    private void runRepeatBlock(String group, Runnable callback) {
        Orchestrator command = new Orchestrator();
        command.run(callback::run);
        addStack(group, command, true);
        callback.run();
    }

    private void runCommandBlock(String group, Command command) {
        addStack(group, command, false);
        command.getRequirements().clear();
    }

    private void addLabel(String name, int destination) {
        labels.put(name, destination);
    }

    private void cancelStack(RunStack stack) {
        if (stack.command.isFinished()) {
            return;
        }
        stack.command.end(true);
    }

    private double getTime() {
        return MathSharedStore.getTimestamp();
    }
}

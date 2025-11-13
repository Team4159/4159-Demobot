package frc.robot.lib;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

/*
 * rudimentary library for rumble feedback on controllers and HIDs
 */
public class HIDRumble {
    private static final double kDefaultRequestDuration = Units.millisecondsToSeconds(50);
    private static final int kDefaultRequestPriority = 0;
    private static final boolean kRumblePersistWhileDisabled = false;

    private static boolean rumbleEnabled = true;

    private static final HashMap<GenericHID, RumbleManager> rumbleManagerMap = new HashMap<>();

    static {
        new RumbleLooper();
    };

    private HIDRumble() {
    }

    public static void rumble(GenericHID hid, RumbleRequest rumbleRequest) {
        RumbleManager existingRumbleManager = rumbleManagerMap.get(hid);
        RumbleManager rumbleManager = (existingRumbleManager != null) ? existingRumbleManager : new RumbleManager(hid);
        rumbleManager.request(rumbleRequest);
    }

    public static void rumble(CommandGenericHID commandHid, RumbleRequest rumbleRequest) {
        rumble(commandHid.getHID(), rumbleRequest);
    }

    public static void enable(boolean enabled) {
        rumbleEnabled = enabled;
    }

    private static class RumbleLooper extends SubsystemBase {
        @SuppressWarnings("unused")
        private static final RumbleLooper instance = new RumbleLooper();

        private RumbleLooper() {
        }

        @Override
        public void periodic() {
            // update all rumble managers
            for (Map.Entry<GenericHID, RumbleManager> rumbleManagerEntry : rumbleManagerMap.entrySet()) {
                RumbleManager rumbleManager = rumbleManagerEntry.getValue();
                rumbleManager.update();
            }
        }
    }

    private static class RumbleManager {
        private final GenericHID hid;
        private ArrayList<RumbleRequest> rumbleRequestList = new ArrayList<>();
        private int highestPriorityRequestIndex = 0;

        public RumbleManager(GenericHID hid) {
            this.hid = hid;
            HIDRumble.rumbleManagerMap.put(hid, this);
        }

        public void request(RumbleRequest rumbleRequest) {
            rumbleRequestList.add(rumbleRequest);
            if (rumbleRequest.getPriority() > highestPriorityRequestIndex) {
                highestPriorityRequestIndex = rumbleRequest.getPriority();
            }
        }

        public void update() {
            boolean robotEnabled = DriverStation.isEnabled();

            if (!robotEnabled && !kRumblePersistWhileDisabled) {
                rumbleRequestList.clear();
                highestPriorityRequestIndex = 0;
            } else {
                Iterator<RumbleRequest> removeIterator = rumbleRequestList.iterator();
                boolean removedHighestPriorityRequest = false;
                while (removeIterator.hasNext()) {
                    RumbleRequest rumbleRequest = removeIterator.next();
                    if (rumbleRequest.isExpired()) {
                        removeIterator.remove();
                        if (rumbleRequest.getPriority() == highestPriorityRequestIndex) {
                            removedHighestPriorityRequest = true;
                        }
                    }
                }
                if (removedHighestPriorityRequest) {
                    updateHighestPriorityRequestIndex();
                }
            }

            if (rumbleEnabled && rumbleRequestList.size() > 0) {
                setRumbleFromRequest(getLatestHighestPriorityRequest());
            } else {
                hid.setRumble(RumbleType.kBothRumble, 0);
            }
        }

        private void setRumbleFromRequest(RumbleRequest rumbleRequest) {
            RumbleType rumbleType = rumbleRequest.getRumbleType();
            double strength = rumbleRequest.getStrength();

            double leftStrength = 0, rightStrength = 0;
            switch (rumbleType) {
                case kLeftRumble:
                    leftStrength = strength;
                    break;
                case kRightRumble:
                    rightStrength = strength;
                    break;
                case kBothRumble:
                    leftStrength = strength;
                    rightStrength = strength;
                    break;
            }
            hid.setRumble(RumbleType.kLeftRumble, leftStrength);
            hid.setRumble(RumbleType.kRightRumble, rightStrength);
        }

        private void updateHighestPriorityRequestIndex() {
            highestPriorityRequestIndex = 0;
            for (RumbleRequest rumbleRequest : rumbleRequestList) {
                if (rumbleRequest.getPriority() > highestPriorityRequestIndex) {
                    highestPriorityRequestIndex = rumbleRequest.getPriority();
                }
            }
        }

        private RumbleRequest getLatestHighestPriorityRequest() {
            for (int i = rumbleRequestList.size() - 1; i >= 0; i--) {
                RumbleRequest rumbleRequest = rumbleRequestList.get(i);
                if (rumbleRequest.getPriority() == highestPriorityRequestIndex) {
                    return rumbleRequest;
                }
            }
            return null;
        }
    }

    public static class RumbleRequest implements Cloneable {
        private final double start, lifespan, strength;
        private final RumbleType rumbleType;
        private final int priority;

        public RumbleRequest(RumbleType rumbleType, double strength, double lifespan, int priority) {
            this.start = Timer.getFPGATimestamp();
            this.rumbleType = rumbleType;
            this.strength = MathUtil.clamp(strength, 0, 1);
            this.lifespan = Math.max(0, lifespan);
            this.priority = priority;
        }

        public RumbleRequest(RumbleType rumbleType, double strength) {
            this(rumbleType, strength, kDefaultRequestDuration, kDefaultRequestPriority);
        }

        public RumbleRequest(double strength) {
            this(RumbleType.kBothRumble, strength, kDefaultRequestDuration, kDefaultRequestPriority);
        }

        public RumbleRequest(RumbleType rumbleType, double strength, int priority) {
            this(rumbleType, strength, kDefaultRequestDuration, priority);
        }

        public RumbleRequest(double strength, int priority) {
            this(RumbleType.kBothRumble, strength, kDefaultRequestDuration, priority);
        }

        public boolean isExpired() {
            return Timer.getFPGATimestamp() - start > lifespan;
        }

        public RumbleType getRumbleType() {
            return rumbleType;
        }

        public double getStrength() {
            return strength;
        }

        public int getPriority() {
            return priority;
        }
    }
}

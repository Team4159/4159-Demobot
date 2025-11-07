package frc.robot.lib;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

/*
 * rudimentary library for rumble feedback on controllers and HIDs
 */
public class HIDRumble extends SubsystemBase {
    private static final double kDefaultRequestDuration = 0.1;
    private static final int kDefaultRequestPriority = 0;
    private static final HashMap<GenericHID, RumbleState> rumbleStateMap = new HashMap<>();


    @SuppressWarnings("unused")
    private static HIDRumble instance = new HIDRumble();

    private HIDRumble() {}

    public static RumbleState getHIDState(GenericHID hid) {
        return rumbleStateMap.get(hid);
    }

    public static RumbleState getHIDState(CommandGenericHID commandHid) {
        return getHIDState(commandHid.getHID());
    }

    @Override
    public void periodic() {
        for (Map.Entry<GenericHID, RumbleState> rumbleStatePair : rumbleStateMap.entrySet()) {
            rumbleStatePair.getValue().update();
        }
    }

    public static class RumbleState {
        private final GenericHID hid;
        private ArrayList<RumbleRequest> rumbleRequestList = new ArrayList<>();
        private int highestPriorityRequestIndex = 0;

        public RumbleState(GenericHID hid) {
            this.hid = hid;
            HIDRumble.rumbleStateMap.put(hid, this);
        }

        public RumbleState(CommandGenericHID hid) {
            this(hid.getHID());
        }

        public void request(RumbleRequest rumbleRequest) {
            rumbleRequestList.add(rumbleRequest);
            if (rumbleRequest.getPriority() > highestPriorityRequestIndex) {
                highestPriorityRequestIndex = rumbleRequest.getPriority();
            }
        }

        public void update() {
            
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
            
            double leftStrength = 0, rightStrength = 0;

            if (rumbleRequestList.size() > 0) {
                RumbleRequest latestRumbleRequest = getLatestHighestPriorityRequest();
                RumbleType rumbleType = latestRumbleRequest.getRumbleType();
                double strength = latestRumbleRequest.getStrength();
                
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

        public boolean isExpired() { return Timer.getFPGATimestamp() - start > lifespan; }

        public RumbleType getRumbleType() { return rumbleType; }

        public double getStrength() { return strength; }

        public int getPriority() { return priority; }
    }
}

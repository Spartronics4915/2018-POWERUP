package com.team254.lib.util.drivers;

import com.spartronics4915.lib.util.drivers.TalonSRX4915Factory;
import com.spartronics4915.lib.util.drivers.TalonSRX4915;

import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mockito;
import org.mockito.invocation.InvocationOnMock;
import org.mockito.stubbing.Answer;
import org.powermock.api.mockito.PowerMockito;
import org.powermock.core.classloader.annotations.PrepareForTest;
import org.powermock.modules.junit4.PowerMockRunner;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

@RunWith(PowerMockRunner.class)
@PrepareForTest(TalonSRX4915Factory.class)
public class CANTalonFactoryTest {

    @Test
    public void testWhichMethodsAreCalled() throws Exception {
        List<String> acceptableUncalledMethodNames = Arrays.asList(
                "getClass",
                "pidWrite",
                "wait",
                "notifyAll",
                "delete",
                "disableControl",
                "notify",
                "setF",
                "setI",
                "hashCode",
                "enable",
                "setD",
                "setParameter",
                "setP",
                "setPIDSourceType",
                "initTable",
                "getParameter",
                "getMotionProfileStatus",
                "startLiveWindowMode",
                "enableControl",
                "updateTable",
                "setSetpoint",
                "GetGadgeteerStatus",
                "disable",
                "pushMotionProfileTrajectory",
                "equals",
                "reset",
                "stopMotor",
                "processMotionProfileBuffer",
                "setControlMode",
                "stopLiveWindowMode",
                "getMotionMagicActTrajPosition",
                "getMotionMagicActTrajVelocity",
                "DisableNominalClosedLoopVoltage",
                "createTableListener");

        final Set<String> uncalledMethodNames = new HashSet<>(
                Arrays.stream(TalonSRX4915.class.getMethods())
                        .map(m -> m.getName())
                        .filter(name -> !acceptableUncalledMethodNames.contains(name))
                        .collect(Collectors.toSet()));

        TalonSRX4915 talon = Mockito.mock(TalonSRX4915.class, new Answer() {
            @Override
            public Object answer(InvocationOnMock invocationOnMock) throws Throwable {
                uncalledMethodNames.remove(invocationOnMock.getMethod().getName());
                return null;
            }
        });
        PowerMockito.whenNew(TalonSRX4915.class).withAnyArguments().thenReturn(talon);

        TalonSRX4915 returnedTalon = TalonSRX4915Factory.createDefaultMotor(1);
        String talonInfo = TalonSRX4915Factory.getFullTalonInfo(returnedTalon);

        Assert.assertEquals(
                new HashSet<>(),
                uncalledMethodNames);
    }

    @Test
    public void testCanPrintInfo() {
        System.out.println(TalonSRX4915Factory.getFullTalonInfo(Mockito.mock(TalonSRX4915.class)));
    }
}

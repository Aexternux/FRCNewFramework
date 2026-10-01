package frc.robot.application.auto;

import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

import frc.robot.application.auto.routines.PlannedAuto;
import frc.robot.application.auto.routines.ReferenceAutos;

/** Holds selectable autonomous routines. */
public class AutoSelector {
    private final ArrayList<ActionList> routines = new ArrayList<>();
    private int index;

    public AutoSelector() {
        routines.add(PlannedAuto.build());
        routines.add(ReferenceAutos.leftAuto1());
        routines.add(ReferenceAutos.rightAuto1());
        routines.add(ReferenceAutos.leftDisrupt());
        routines.add(ReferenceAutos.rightDisrupt());
    }

    public void add(ActionList routine) { routines.add(routine); }
    public void nextAuto() { index = (index + 1) % routines.size(); }
    public ActionList getSelected() { return routines.get(index); }

    public void periodic() {
        ActionList selected = getSelected();
        Logger.recordOutput("Elastic/Selected Auto/Name", selected.getName());
        Logger.recordOutput("Elastic/Selected Auto/Index", index);
        Logger.recordOutput("Elastic/Selected Auto/Start", selected.getStartDescription());
    }
}

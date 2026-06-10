package dev.ryanhcode.sable.mixinterface.plot;

import dev.ryanhcode.sable.api.sublevel.SubLevelContainer;
import org.jetbrains.annotations.ApiStatus;

@ApiStatus.Internal
public interface SubLevelContainerHolder {

    SubLevelContainer sable$getPlotContainer();

}

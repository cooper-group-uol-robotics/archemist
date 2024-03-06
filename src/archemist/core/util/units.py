from unyt import Unit, UnitRegistry
from unyt.dimensions import volume

reg = UnitRegistry()
reg.add(symbol="L", base_value=1e-3, dimensions=volume, tex_repr=r"\rm{L}",
        prefixable=True)

m3 = Unit("m**3", registry=reg)
cm3 = Unit("cm**3", registry=reg)
mm3 = Unit("mm**3", registry=reg)

L = Unit("L", registry=reg)
mL = Unit("mL", registry=reg)
uL = Unit("uL", registry=reg)

kg = Unit("kg", registry=reg)
g = Unit("g", registry=reg)
mg = Unit("mg", registry=reg)
ug = Unit("ug", registry=reg)

second = Unit("s", registry=reg)
minute = Unit("min", registry=reg)
hour = Unit("hr", registry=reg)

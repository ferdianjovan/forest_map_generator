from datetime import date
import sys

import numpy as np

try:
    from wmm import wmm_calc
except ModuleNotFoundError as exc:
    raise ModuleNotFoundError(
        "Missing Python package 'wmm-calculator'. ROS is running this node with "
        f"'{sys.executable}', so install it for that interpreter, for example: "
        f"{sys.executable} -m pip install wmm-calculator"
    ) from exc


def calculate_gazebo_magnetic_field(
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    calculation_date: date | None = None,
) -> tuple[float, float, float]:
    """
    Calculate the magnetic-field vector for a Gazebo ENU world.

    Returns:
        (east_tesla, north_tesla, up_tesla)
    """
    model = wmm_calc()

    # Altitude is supplied in metres above mean sea level.
    model.setup_env(
        latitude_deg,
        longitude_deg,
        altitude_m,
        unit="m",
        msl=True,
    )

    calculation_date = calculation_date or date.today()

    model.setup_time(
        calculation_date.year,
        calculation_date.month,
        calculation_date.day,
    )

    field = model.get_all()

    # WMM output:
    # x = north, y = east, z = down, all in nanotesla.
    north_nt = float(np.asarray(field["x"]).squeeze())
    east_nt = float(np.asarray(field["y"]).squeeze())
    down_nt = float(np.asarray(field["z"]).squeeze())

    # Gazebo ENU:
    # x = east, y = north, z = up.
    # Convert nanotesla to tesla.
    east_t = east_nt * 1e-9
    north_t = north_nt * 1e-9
    up_t = -down_nt * 1e-9

    return east_t, north_t, up_t

def generate_world_geography(latitude: float, longitude: float, altitude: float) -> str:
    east, north, up = calculate_gazebo_magnetic_field(
        latitude,
        longitude,
        altitude,
    )

    return f"""
<magnetic_field>{east:.10e} {north:.10e} {up:.10e}</magnetic_field>

<spherical_coordinates>
  <surface_model>EARTH_WGS84</surface_model>
  <world_frame_orientation>ENU</world_frame_orientation>
  <latitude_deg>{latitude}</latitude_deg>
  <longitude_deg>{longitude}</longitude_deg>
  <elevation>{altitude}</elevation>
  <heading_deg>0</heading_deg>
</spherical_coordinates>
""".strip()


if __name__ == "__main__":
    latitude = 47.397971057728974
    longitude = 8.546163739800146
    altitude = 65.0

    world_geography = generate_world_geography(
        latitude=latitude,
        longitude=longitude,
        altitude=altitude,
    )

    print(world_geography)

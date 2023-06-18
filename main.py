import xml.etree.ElementTree as et
from sys import argv
from math import atan, sqrt


def read_den_har(path):
    ans = []
    with open(path, "r", encoding="utf-8") as file:
        for i in file:
            ans.append(tuple(map(float, i[:-1].split())))
    return ans


def split_data(data):
    coords = []
    limits = []
    damping = []
    t = []

    for i in data:
        coords.append(i[:4])
        limits.append(i[4:-2])
        damping.append(i[-2])
        t.append(i[-1])

    return coords, limits, damping, t


def generate_urdf(data):
    robot = et.Element("robot", name="my_robot")

    last_name = "world"
    et.SubElement(robot, "link", name=last_name)

    for idx, i in enumerate(data):
        limits, dh_data, damping, t = i

        t = int(t)

        link_name, joint_name, joint_view_name, static_name = (
            f"link_{idx}",
            f"joint_{idx}",
            f"joint_view_{idx}",
            f"static_{idx}",
        )
        print(limits, link_name, joint_name, dh_data)

        static_join = et.SubElement(robot, "joint", name=static_name, type="fixed")
        et.SubElement(static_join, "parent", link=last_name)
        et.SubElement(static_join, "child", link=link_name)
        et.SubElement(static_join, "origin", xyz="0 0 0", rpy=f"0 0 {dh_data[3]}")

        cur_link = et.SubElement(robot, "link", name=link_name)
        cur_visual = et.SubElement(cur_link, "visual")
        et.SubElement(
            et.SubElement(cur_visual, "geometry"),
            "cylinder",
            radius="0.02",
            length=f"{sqrt(dh_data[2] **2  + dh_data[0] ** 2)}",
        )
        et.SubElement(
            cur_visual,
            "origin",
            xyz=f"{dh_data[0]/2} 0 {dh_data[2]/2}",
            rpy=f"0 {atan(dh_data[0]/dh_data[2]) if dh_data[2] != 0 else 0} 0",
        )

        cur_joint = et.SubElement(
            robot, "joint", name=joint_name, type="revolute" if t == 0 else "prismatic"
        )
        et.SubElement(cur_joint, "parent", link=link_name)
        et.SubElement(cur_joint, "child", link=joint_view_name)
        et.SubElement(cur_joint, "axis", xyz="0 0 1")
        et.SubElement(cur_joint, "limit", lower=str(limits[0]), upper=str(limits[1]))
        et.SubElement(
            cur_joint,
            "origin",
            xyz=f"{dh_data[0]} 0 {dh_data[2]}",
            rpy=f"{dh_data[1]} 0 0",
        )
        et.SubElement(cur_joint, "dynamics", damping=str(damping))

        cur_coint_view = et.SubElement(robot, "link", name=joint_view_name)
        cur_visual = et.SubElement(cur_coint_view, "visual")
        et.SubElement(
            et.SubElement(cur_visual, "geometry"),
            "cylinder",
            radius="0.05",
            length="0.1",
        )

        if t == 1:
            tmp_name = f"prismatic_view_{idx}"
            cur_joint = et.SubElement(robot, "joint", name=f"prismatic_joint_{idx}", type="fixed")
            et.SubElement(cur_joint, "parent", link=joint_view_name)
            et.SubElement(cur_joint, "child", link=tmp_name)

            tmp = et.SubElement(robot, "link", name=tmp_name)
            cur_visual = et.SubElement(tmp, "visual")
            et.SubElement(
                et.SubElement(cur_visual, "geometry"),
                "cylinder",
                radius="0.02",
                length=f"{limits[1]}",
            )
            et.SubElement(cur_visual, "origin", xyz=f"0 0 -{limits[1]/2}")
            et.SubElement(
                et.SubElement(cur_visual, "material", name="a"),
                "color",
                rgba="30 45 60 0.2",
            )

        last_name = joint_view_name

    final = et.SubElement(robot, "joint", name="joint_final", type="fixed")
    et.SubElement(final, "parent", link=last_name)
    et.SubElement(final, "child", link="link_final")

    final = et.SubElement(robot, "link", name="link_final")
    cur_visual = et.SubElement(final, "visual")
    et.SubElement(et.SubElement(cur_visual, "geometry"), "box", size="0.05 0.05 0.1")
    et.SubElement(cur_visual, "origin", xyz="0 0 0.1")

    return robot


if __name__ == "__main__":
    input_file, output_file = argv[1], argv[2] if len(argv) == 3 else "./out.xml"
    dh_data, limits, damping, t = split_data(read_den_har(input_file))
    robot = generate_urdf(zip(limits, dh_data, damping, t))
    et.indent(robot)
    et.ElementTree(robot).write(output_file)

import xml.etree.ElementTree as ET

def generate_mujoco_xml(num_bodies):
    root = ET.Element("mujoco", model="my_model")
    include = ET.SubElement(root, "include", file="scene.xml")

    worldbody = ET.SubElement(root, "worldbody")
    # actuator =  ET.SubElement(root, "actuator")
    pos = [0,0,1]
    quat = [0, -0.707,0, 0.707]

    for i in range(num_bodies):
        pos[0] += 0.02
        body = ET.SubElement(worldbody, "body", name=f"object{i+1}", pos="{} {} {}".format(*pos), quat="{} {} {} {}".format(*quat))
        composite = ET.SubElement(body, "composite", prefix=f"o{i+1}", type="grid", count="20 1 1", spacing="0.02", offset="0 0 0")
        joint = ET.SubElement(composite, "joint", kind="main", damping="0.001")
        tendon = ET.SubElement(composite, "tendon", kind="main", width="0.001")
        geom = ET.SubElement(composite, "geom", size=".005", rgba=".8 .2 .1 1")
        pin = ET.SubElement(composite, "pin", coord="0")

    tree = ET.ElementTree(root)
    indent(root)
    tree.write("my_model.xml", encoding="utf-8", xml_declaration=True)

def indent(elem, level=0):
    # 들여쓰기를 추가하기 위한 함수
    indent_size = 2
    i = "\n" + level * indent_size * " "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + indent_size * " "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level + 1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

generate_mujoco_xml(100)

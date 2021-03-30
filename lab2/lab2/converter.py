import math

DHtable = []
with open("../config/DHtable.txt") as file:
    for line in file:
        if line[-1] == '\n':
            line = line[0:-1]
        params = line.split()
        DHtable.append(params)

print(DHtable)
DHtable = DHtable[1:]

with open("../robot/robot.urdf.xml", 'w') as file:
    file.write('<robot name="robot">\n\n')
    for i, link in enumerate(DHtable):
        if i != 0:
            prlink = DHtable[i-1]
            
            #joint

            link_type = ""
            if prlink[1] == "x":
                link_type = "prismatic"
                prlink[1] = 0
            elif prlink[3] == "x":
                link_type = "revolute"
                prlink[3] = 0
            else:
                link_type = "fixed"

            file.write(f'<joint name="{prlink[4]}-{link[4]}" type="{link_type}">\n')
            
            file.write(f'  <origin xyz="{prlink[0]} 0 {prlink[1] if prlink[1] != "x" else 0}" rpy="{prlink[2]} 0 {link[3] if link[3] != "x" else 0}"/>\n')
            file.write(f'  <parent link="{prlink[4]}"/>\n')
            file.write(f'  <child link="{link[4]}"/>\n')
            file.write(f'  <axis xyz="0 0 1"/>\n')
            file.write('  <limit upper="0" lower="0" effort="10" velocity="10"/>\n')

            file.write('</joint>\n\n')

        # link
        file.write(f'<link name="{link[4]}">\n')

        file.write('  <visual>\n')
        if link[1] == "x":
            l1 = 0
            link_len = 0
        else:
            l1 = link[1]
            link_len = (float(link[0])**2+float(l1)**2)**0.5
        if link_len != 0:
            angle = math.asin(float(link[0])/link_len)
        else:
            angle = 0
        file.write(f'    <origin xyz="{float(link[0])/2} 0 {float(l1)/2}" rpy="0 {angle} 0"/>\n')
        file.write('      <geometry>\n')
             
        file.write(f'        <cylinder radius="0.05" length="{link_len}"/>\n')
        file.write('      </geometry>\n')
        file.write('      <material name="white">\n')
        file.write('        <color rgba="1 1 1 1"/>\n')
        file.write('      </material>\n')
        file.write('  </visual>\n')

        file.write('</link>\n\n')

    file.write('</robot>\n')
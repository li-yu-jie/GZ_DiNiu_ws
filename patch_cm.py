import re

with open('/home/y/computer/Docker/ubuntu22-rootfs/home/y/GZ_DiNiu_ws/src/diuniu_n10_nav/config/nav2_params.yaml', 'r') as f:
    content = f.read()

# 1. Update collision_monitor to use /scan_filtered
content = re.sub(r'topic: "/scan"(.*?#.*)', r'topic: "/scan_filtered"\1', content)

# 2. Update FrontStop from [1.65, 1.85] to [1.95, 2.15]
content = re.sub(r'points: \[1\.65, -0\.34, 1\.65, 0\.34, 1\.85, 0\.34, 1\.85, -0\.34\]', r'points: [1.95, -0.34, 1.95, 0.34, 2.15, 0.34, 2.15, -0.34]', content)

# 3. Update BodyStop from [1.65, -0.38] to [1.95, -0.38] (front boundary)
content = re.sub(r'points: \[1\.65, -0\.43, 1\.65, 0\.43, -0\.38, 0\.43, -0\.38, -0\.43\]', r'points: [1.95, -0.43, 1.95, 0.43, -0.38, 0.43, -0.38, -0.43]', content)

# 4. Update SideStop front boundary from 1.60 to 1.90
content = re.sub(r'points: \[1\.60, 0\.43, 1\.60, 0\.75, -0\.30, 0\.75, -0\.30, 0\.43,\n\s*-0\.30, -0\.43, -0\.30, -0\.75, 1\.60, -0\.75, 1\.60, -0\.43\]', r'points: [1.90, 0.43, 1.90, 0.75, -0.30, 0.75, -0.30, 0.43,\n               -0.30, -0.43, -0.30, -0.75, 1.90, -0.75, 1.90, -0.43]', content)

# 5. Update FrontSlowdown from [1.62, 2.40] to [1.92, 2.70]
content = re.sub(r'points: \[1\.62, -0\.34, 1\.62, 0\.34, 2\.40, 0\.34, 2\.40, -0\.34\]', r'points: [1.92, -0.34, 1.92, 0.34, 2.70, 0.34, 2.70, -0.34]', content)

with open('/home/y/computer/Docker/ubuntu22-rootfs/home/y/GZ_DiNiu_ws/src/diuniu_n10_nav/config/nav2_params.yaml', 'w') as f:
    f.write(content)

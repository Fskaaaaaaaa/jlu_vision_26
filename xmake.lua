set_project("vision")
set_version("0.0.1")
add_rules("mode.debug", "mode.release")
set_languages("c++20")

rule("iceoryx_deps")
on_load(function(target)
	target:add("includedirs", "/usr/local/include/iceoryx/v")
	target:add("linkdirs", "/usr/local/lib")
	target:add("links", "iceoryx_posh", "iceoryx_hoofs", "iceoryx_platform")
end)
rule_end()

rule("openvino_deps")
on_load(function(target)
	target:add("includedirs", "/opt/intel/openvino_2024/runtime/include")
	target:add("linkdirs", "/opt/intel/openvino_2024/runtime/lib/intel64")
	target:add("links", "openvino")
end)
rule_end()

rule("open3d_deps")
on_load(function(target)
	local open3d_root = path.absolute("src/thirdparty/open3d/open3d-devel-linux-x86_64-cxx11-abi-0.19.0")
	target:add("includedirs", path.join(open3d_root, "include"))
	target:add("linkdirs", path.join(open3d_root, "lib"))
	target:add("links", "Open3D")
	target:add("rpathdirs", path.join(open3d_root, "lib"))
end)

rule("hik_deps")
on_load(function(target)
	local hik_root = path.absolute("src/thirdparty/hik_camera_sdk")
	target:add("includedirs", path.join(hik_root, "include"))
	if is_arch("x86_64") then
		target:add("linkdirs", path.join(hik_root, "lib/amd64"))
	else
		target:add("linkdirs", path.join(hik_root, "lib/arm64"))
	end
	target:add("links", "MvCameraControl")
end)
rule_end()

add_requires("opencv")
add_requires("eigen")
add_requires("yaml-cpp")
add_requires("magic_enum")
add_requires("nameof")
add_requires("g2o")
-- FIXME: g2o官方package的deps缺少fmt和spdlog导致链接不通过,
-- 得手动添加到～/.xmake/repositories/xmake-repo/packages/g/g2o/xmake.lua
add_requires("glfw")

add_subdirs("src/msgs")
add_subdirs("src/tools")
add_subdirs("src/hardware")
add_subdirs("src/auto_aim")
add_subdirs("src/auto_buff")

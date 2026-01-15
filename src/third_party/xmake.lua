rule("open3d_deps")
on_load(function(target)
	local open3d_root = path.join(os.scriptdir(), "open3d/open3d-devel-linux-x86_64-cxx11-abi-0.19.0")
	target:add("includedirs", path.join(open3d_root, "include"))
	target:add("linkdirs", path.join(open3d_root, "lib"))
	target:add("links", "Open3D")
	target:add("rpathdirs", path.join(open3d_root, "lib"))
end)
-- NOTE: 注意open3d依赖clang工具链的libc++，记得装一下
-- 另外open3d的二进制库太大了，传不了github，拉取构建的时候自己下吧
rule_end()

rule("hik_deps")
on_load(function(target)
	local sdk_root = path.join(os.scriptdir(), "hik_camera_sdk")
	target:add("includedirs", path.join(sdk_root, "include"))
	if is_arch("x86_64") then
		target:add("linkdirs", path.join(sdk_root, "lib/amd64"))
	else
		target:add("linkdirs", path.join(sdk_root, "lib/arm64"))
	end
	target:add("links", "MvCameraControl")
end)
rule_end()

target("tinympc")
set_kind("static")
add_files("tinympc/*.cpp")
add_includedirs("tinympc", { public = true })
add_packages("eigen")
target_end()

target("fast_tf")
set_kind("static")
add_files("fast_tf/*.cpp")
add_includedirs("fast_tf/fast_tf", { public = true })
add_packages("eigen")
target_end()

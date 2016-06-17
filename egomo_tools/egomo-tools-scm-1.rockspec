package = "egomo-tools"
version = "scm-1"

source = {
   url = "git://github.com/Xamla/prototyping/tree/master/westerhoff/lua-modules",
}

description = {
   summary = "Collection of client classes providing simpilfied sensor access and roboter contol",
   detailed = [[
   ]],
   homepage = "https://github.com/Xamla/prototyping/tree/master/westerhoff/lua-modules",
   license = "properitary"
}

dependencies = {
   "torch >= 7.0",
   "md5 >= 1.2-1"
}

build = {
   type = "command",
   build_command = [[
cmake -E make_directory build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$(PREFIX)" && $(MAKE)
]],
   install_command = "cd build && $(MAKE) install"
}

# Copyright 2024 Mikhail Gozhev <m@gozhev.org>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys

from argparse import ArgumentParser
import stat
import logging
import errno
import json
import time

import asyncio
import pyfuse3

from ros2_runtime_fs.ros_graph import node_is_hidden, ROSGraph, service_is_hidden, topic_is_hidden

try:
    import pyfuse3.asyncio
    pyfuse3.asyncio.enable()
except:
    import pyfuse3_asyncio
    pyfuse3_asyncio.enable()

try:
    import faulthandler
except ImportError:
    pass
else:
    faulthandler.enable()


log = logging.getLogger(__name__)


class RosRuntimeFs(pyfuse3.Operations):

    def attr_entry_link(self, inode):
        entry = pyfuse3.EntryAttributes()
        entry.st_mode = (stat.S_IFLNK | 0o755)
        entry.st_size = 0
        stamp = int(1438467123.985654 * 1e9)
        entry.st_atime_ns = stamp
        entry.st_ctime_ns = stamp
        entry.st_mtime_ns = stamp
        entry.st_gid = os.getgid()
        entry.st_uid = os.getuid()
        entry.st_ino = inode
        return entry

    def attr_entry_file(self, inode):
        entry = pyfuse3.EntryAttributes()
        entry.st_mode = (stat.S_IFREG | 0o644)
        entry.st_size = 0
        stamp = int(1438467123.985654 * 1e9)
        entry.st_atime_ns = stamp
        entry.st_ctime_ns = stamp
        entry.st_mtime_ns = stamp
        entry.st_gid = os.getgid()
        entry.st_uid = os.getuid()
        entry.st_ino = inode
        return entry

    def attr_entry_dir(self, inode):
        entry = pyfuse3.EntryAttributes()
        entry.st_mode = (stat.S_IFDIR | 0o755)
        entry.st_size = 0
        stamp = int(1438467123.985654 * 1e9)
        entry.st_atime_ns = stamp
        entry.st_ctime_ns = stamp
        entry.st_mtime_ns = stamp
        entry.st_gid = os.getgid()
        entry.st_uid = os.getuid()
        entry.st_ino = inode
        return entry

    def add_inode_entry(self, inode, attr_entry, data):
        self.inode_table.update({inode: (attr_entry, data)})

    def add_tree_entry(self, inode, name, parent_tree):
        parent_tree.update({name: inode})

    def add_link(self, parent_tree, name, target):
        data = target.encode("utf-8")
        inode = self.next_inode
        attr_entry = self.attr_entry_link(inode)
        attr_entry.st_size = len(data)
        self.add_inode_entry(inode, attr_entry, data)
        self.add_tree_entry(inode, name, parent_tree)
        self.next_inode += 1

    def add_file(self, parent_tree, name, data):
        inode = self.next_inode
        attr_entry = self.attr_entry_file(inode)
        attr_entry.st_size = len(data)
        self.add_inode_entry(inode, attr_entry, data)
        self.add_tree_entry(inode, name, parent_tree)
        self.next_inode += 1

    def add_dir(self, parent_tree, name, data):
        inode = self.next_inode
        attr_entry = self.attr_entry_dir(inode)
        self.add_inode_entry(inode, attr_entry, data)
        self.add_tree_entry(inode, name, parent_tree)
        self.next_inode += 1

    def dir_at(self, parent_tree, ns_list):
        cur_tree = parent_tree
        for ns in ns_list:
            try:
                next_tree_inode = cur_tree[ns]
                _, next_tree = self.inode_table[next_tree_inode]
            except KeyError:
                next_tree = {}
                self.add_dir(cur_tree, ns, next_tree)
            cur_tree = next_tree
        return cur_tree

    def path_to_nth_parent(self, n):
        return "/".join(([".."] * n) + [""])

    def add_conn_to_graph(self, conn, conn_type):
        conn_name = conn.conn_name
        conn_ns_list = list(filter(None, conn_name.split("/"))) + [conn_type]
        conn_tree = self.dir_at(self.graph, conn_ns_list) 
        if "name" not in conn_tree:
            self.add_file(conn_tree, "name", (conn_name + "\n").encode("utf-8"))
        return conn_tree, conn_ns_list

    def proc_node_conn(self, node_tree, node_ns_list, conn_list, conn_type, link_type):
        if not conn_list:
            return
        node_to_root_path = self.path_to_nth_parent(len(node_ns_list) + 1)
        node_link_dir = self.dir_at(node_tree, [link_type])
        for conn in conn_list:
            conn_tree, conn_ns_list = self.add_conn_to_graph(conn, conn_type)
            conn_link_dir = self.dir_at(conn_tree, [link_type])
            conn_to_root_path = self.path_to_nth_parent(len(conn_ns_list) + 1)

            link_name = "\\".join(conn_ns_list[:-1])
            link_target = node_to_root_path + "/".join(conn_ns_list)
            self.add_link(node_link_dir, link_name, link_target)

            link_name = "\\".join(node_ns_list[:-1])
            link_target = conn_to_root_path + "/".join(node_ns_list)
            self.add_link(conn_link_dir, link_name, link_target)

    def add_node_to_graph(self, node_name, node):
        node_ns_list = list(filter(None, node_name.split("/"))) + ["NODE"]
        node_tree = self.dir_at(self.graph, node_ns_list) 
        self.add_file(node_tree, "name", (node_name + "\n").encode("utf-8"))

        self.proc_node_conn(node_tree, node_ns_list, node.topic_publishers, "TOPIC", "publishers")
        self.proc_node_conn(node_tree, node_ns_list, node.topic_subscribers, "TOPIC", "subscribers")
        self.proc_node_conn(node_tree, node_ns_list, node.service_clients, "SERVICE", "service_clients")
        self.proc_node_conn(node_tree, node_ns_list, node.service_servers, "SERVICE", "service_servers")
        self.proc_node_conn(node_tree, node_ns_list, node.action_clients, "ACTION", "action_clients")
        self.proc_node_conn(node_tree, node_ns_list, node.action_servers, "ACTION", "action_servers")

        return node_tree, node_ns_list

    def __init__(self):
        super(RosRuntimeFs, self).__init__()
        
        self.root = {}
        self.graph = {}

        self.inode_table = {}
        self.add_inode_entry(pyfuse3.ROOT_INODE, self.attr_entry_dir(pyfuse3.ROOT_INODE), self.root)

        self.next_inode = pyfuse3.ROOT_INODE + 1

        self.add_file(self.root, "version", b"ros2_runtime_fs 0.1.0\n")
        self.add_dir(self.root, "graph", self.graph)
        self.add_file(self.root, "update_graph", b"updating the graph...\nok\n")

        self.ros_network = ROSGraph()
        time.sleep(5)
        self.node_list = self.ros_network.get_nodes()

        for name, node in self.node_list.items():
            self.add_node_to_graph(name, node)

    def __del__(self):
        self.ros_network.shutdown()

    async def getattr(self, inode, ctx=None):
        try:
            entry, _ = self.inode_table[inode]
            return entry
        except:
            raise pyfuse3.FUSEError(errno.ENOENT)

    async def lookup(self, parent_inode, name, ctx=None):
        try:
            _, tree = self.inode_table[parent_inode]
            inode = tree[name.decode("utf-8")]
            entry, _ = self.inode_table[inode]
            return entry
        except:
            raise pyfuse3.FUSEError(errno.ENOENT)

    async def opendir(self, inode, ctx):
        if inode not in self.inode_table:
            raise pyfuse3.FUSEError(errno.ENOENT)
        return inode

    async def readdir(self, fh, start_id, token):
        _, tree = self.inode_table[fh]
        cur_id = 0
        next_id = 0
        for name, inode in tree.items():
            cur_id = next_id
            next_id = cur_id + 1
            if cur_id < start_id:
                continue
            pyfuse3.readdir_reply(
                    token, name.encode("utf-8"), await self.getattr(inode), next_id)

    async def open(self, inode, flags, ctx):
        if inode not in self.inode_table:
            raise pyfuse3.FUSEError(errno.ENOENT)
        #if flags & os.O_RDWR or flags & os.O_WRONLY:
        #    raise pyfuse3.FUSEError(errno.EACCES)
        return pyfuse3.FileInfo(fh=inode)

    async def read(self, fh, off, size):
        _, data = self.inode_table[fh]
        return data[off:off+size]

    async def readlink(self, inode, ctx=None):
        _, data = self.inode_table[inode]
        return data

    async def setxattr(self, inode, name, value, ctx):
        if inode != pyfuse3.ROOT_INODE or name != b'command':
            raise pyfuse3.FUSEError(errno.ENOTSUP)

        if value == b'terminate':
            pyfuse3.terminate()
        else:
            raise pyfuse3.FUSEError(errno.EINVAL)

def init_logging(debug=False):
    formatter = logging.Formatter('%(asctime)s.%(msecs)03d %(threadName)s: '
                                  '[%(name)s] %(message)s', datefmt="%Y-%m-%d %H:%M:%S")
    handler = logging.StreamHandler()
    handler.setFormatter(formatter)
    if debug:
        handler.setLevel(logging.DEBUG)
        log.setLevel(logging.DEBUG)
    else:
        handler.setLevel(logging.INFO)
        log.setLevel(logging.INFO)
    log.addHandler(handler)


def parse_args():
    parser = ArgumentParser()

    parser.add_argument('mountpoint', type=str,
                        help='Where to mount the file system')
    parser.add_argument('--debug', action='store_true', default=False,
                        help='Enable debugging output')
    parser.add_argument('--debug-fuse', action='store_true', default=False,
                        help='Enable FUSE debugging output')
    return parser.parse_args()


def main():
    options = parse_args()
    init_logging(options.debug)
    ros_runtime_fs = RosRuntimeFs()
    fuse_options = set(pyfuse3.default_options)
    fuse_options.add('fsname=ros_runtime_fs')
    if options.debug_fuse:
        fuse_options.add('debug')
    pyfuse3.init(ros_runtime_fs, options.mountpoint, fuse_options)
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(pyfuse3.main())
    except KeyboardInterrupt:
        pass
    finally:
        loop.close()
        pyfuse3.close(unmount=True)


if __name__ == '__main__':
    main()

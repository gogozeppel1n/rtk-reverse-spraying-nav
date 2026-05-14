#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import traceback
import rospy
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger, TriggerRequest

try:
    import tkinter as tk
    from tkinter import messagebox, filedialog
except Exception as e:
    raise RuntimeError('Tkinter 未安装。Ubuntu/ROS1 常见解决办法：sudo apt install python3-tk') from e


class ControllerPanel(object):
    def __init__(self):
        rospy.init_node('controller_panel', anonymous=False)

        ns = rospy.get_param('~nav_ns', '/reverse_track_nav_node')
        self.srv_record_end = rospy.get_param('~srv_record_end', ns + '/record_end_and_start')
        self.srv_record_start = rospy.get_param('~srv_record_start', ns + '/record_start_and_stop')
        self.srv_start_auto = rospy.get_param('~srv_start_auto', ns + '/start_auto_reverse')
        self.srv_stop_auto = rospy.get_param('~srv_stop_auto', ns + '/stop_auto')
        self.srv_clear = rospy.get_param('~srv_clear', ns + '/clear_path')
        self.srv_save = rospy.get_param('~srv_save', ns + '/save_path')
        self.srv_import = rospy.get_param('~srv_import', ns + '/import_path')
        self.srv_pulse_relay = rospy.get_param('~srv_pulse_relay', ns + '/pulse_relay')
        self.import_path_param = rospy.get_param('~import_path_param', ns + '/import_path_file')
        self.default_import_dir = rospy.get_param('~default_import_dir', '/home/agx/rtk_reverse_paths')
        self.srv_set_line = rospy.get_param('~srv_set_line', ns + '/set_line_mode')
        self.srv_set_path = rospy.get_param('~srv_set_path', ns + '/set_path_mode')
        self.status_topic = rospy.get_param('~status_topic', ns + '/status_text')
        self.count_topic = rospy.get_param('~count_topic', ns + '/path_count')

        self.status_text = '等待节点状态...'
        self.path_count = 0

        rospy.Subscriber(self.status_topic, String, self.status_cb, queue_size=10)
        rospy.Subscriber(self.count_topic, Int32, self.count_cb, queue_size=10)

        self.root = tk.Tk()
        self.root.title('RTK 倒车导航控制器')
        self.root.geometry('820x570')
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

        self.info_var = tk.StringVar(value=(
            '推荐操作顺序：\n'
            '1. 先选模式：两点直线 或 多点轨迹\n'
            '2. 车停终点，点【记录终点并开始录轨】\n'
            '3. 遥控正向开到起点\n'
            '4. 到起点后点【记录起点并停止录轨】\n'
            '5. 点【启动自动倒车】'
        ))
        self.status_var = tk.StringVar(value=self.status_text)
        self.count_var = tk.StringVar(value='当前录制点数：0')
        self.import_file_var = tk.StringVar(value='')

        title = tk.Label(self.root, text='RTK 倒车导航控制器', font=('Arial', 18, 'bold'))
        title.pack(pady=10)

        info = tk.Label(self.root, textvariable=self.info_var, justify='left', anchor='w', font=('Arial', 11))
        info.pack(fill='x', padx=16)

        frame = tk.Frame(self.root)
        frame.pack(fill='x', padx=16, pady=16)

        btn_cfg = {'font': ('Arial', 11), 'width': 24, 'height': 2}
        tk.Button(frame, text='切换到两点直线模式', command=lambda: self.call_service_async(self.srv_set_line), bg='#d7f7d7', **btn_cfg).grid(row=0, column=0, padx=8, pady=8)
        tk.Button(frame, text='切换到多点轨迹模式', command=lambda: self.call_service_async(self.srv_set_path), bg='#d7e9ff', **btn_cfg).grid(row=0, column=1, padx=8, pady=8)
        tk.Button(frame, text='1. 记录终点并开始录轨', command=lambda: self.call_service_async(self.srv_record_end), **btn_cfg).grid(row=1, column=0, padx=8, pady=8)
        tk.Button(frame, text='2. 记录起点并停止录轨', command=lambda: self.call_service_async(self.srv_record_start), **btn_cfg).grid(row=1, column=1, padx=8, pady=8)
        tk.Button(frame, text='3. 启动自动倒车', command=lambda: self.call_service_async(self.srv_start_auto), bg='#fff2b3', **btn_cfg).grid(row=2, column=0, padx=8, pady=8)
        tk.Button(frame, text='停止自动', command=lambda: self.call_service_async(self.srv_stop_auto), bg='#ffb3b3', **btn_cfg).grid(row=2, column=1, padx=8, pady=8)
        tk.Button(frame, text='保存当前模式轨迹 CSV', command=lambda: self.call_service_async(self.srv_save), **btn_cfg).grid(row=3, column=0, padx=8, pady=8)
        tk.Button(frame, text='清空轨迹', command=lambda: self.call_service_async(self.srv_clear), bg='#ffe4b3', **btn_cfg).grid(row=3, column=1, padx=8, pady=8)
        tk.Button(frame, text='选择并导入 CSV 轨迹', command=self.choose_and_import_csv, bg='#e8d7ff', **btn_cfg).grid(row=4, column=0, padx=8, pady=8)
        tk.Entry(frame, textvariable=self.import_file_var, font=('Arial', 10), width=45).grid(row=4, column=1, padx=8, pady=8, sticky='we')
        tk.Button(frame, text='测试继电器闭合', command=lambda: self.call_service_async(self.srv_pulse_relay), bg='#d7fff2', **btn_cfg).grid(row=5, column=0, padx=8, pady=8)

        status_frame = tk.LabelFrame(self.root, text='状态', font=('Arial', 11, 'bold'))
        status_frame.pack(fill='both', expand=True, padx=16, pady=8)

        tk.Label(status_frame, textvariable=self.count_var, anchor='w', justify='left', font=('Arial', 11)).pack(fill='x', padx=10, pady=6)
        tk.Label(status_frame, text='当前状态：', anchor='w', justify='left', font=('Arial', 11, 'bold')).pack(fill='x', padx=10)
        self.status_label = tk.Label(
            status_frame,
            textvariable=self.status_var,
            anchor='nw',
            justify='left',
            wraplength=700,
            font=('Arial', 11),
            bg='white',
            relief='sunken',
            padx=8,
            pady=8,
        )
        self.status_label.pack(fill='both', expand=True, padx=10, pady=10)

        self.root.after(100, self.periodic_ros_check)

    def status_cb(self, msg):
        self.status_text = msg.data

    def count_cb(self, msg):
        self.path_count = msg.data

    def periodic_ros_check(self):
        self.status_var.set(self.status_text)
        self.count_var.set('当前录制点数：{}'.format(self.path_count))
        if not rospy.is_shutdown():
            self.root.after(100, self.periodic_ros_check)
        else:
            self.root.quit()

    def choose_and_import_csv(self):
        filename = filedialog.askopenfilename(
            title='选择要导入的倒车轨迹 CSV',
            initialdir=self.default_import_dir,
            filetypes=[('CSV 文件', '*.csv'), ('所有文件', '*.*')]
        )
        if not filename:
            return
        self.import_file_var.set(filename)
        rospy.set_param(self.import_path_param, filename)
        self.call_service_async(self.srv_import)

    def call_service_async(self, service_name):
        threading.Thread(target=self.call_service, args=(service_name,), daemon=True).start()

    def call_service(self, service_name):
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
            proxy = rospy.ServiceProxy(service_name, Trigger)
            resp = proxy(TriggerRequest())
            if resp.success:
                messagebox.showinfo('执行成功', resp.message)
            else:
                messagebox.showwarning('执行失败', resp.message)
        except Exception as e:
            traceback.print_exc()
            messagebox.showerror('服务调用异常', '{}\n\n{}'.format(service_name, e))

    def on_close(self):
        try:
            rospy.signal_shutdown('GUI closed')
        except Exception:
            pass
        self.root.destroy()

    def spin(self):
        self.root.mainloop()


if __name__ == '__main__':
    panel = ControllerPanel()
    panel.spin()

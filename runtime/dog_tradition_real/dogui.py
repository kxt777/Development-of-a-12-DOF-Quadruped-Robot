"""Desktop control panel. Individual entries use the same panel and serial owner."""
from __future__ import annotations

import argparse
import signal
import tkinter as tk
from tkinter import ttk

from dog_runtime import DogController, calibration_problems, load_settings


class DogUI:
    def __init__(self, root, controller):
        self.root, self.controller = root, controller
        self.closing = False
        self._last_events = None
        root.title('DOG · 机械狗控制台'+(' · 离线演练' if controller.dry_run else ''))
        root.geometry('960x720')
        root.minsize(840, 650)
        root.configure(background='#101923')
        style = ttk.Style(root)
        style.theme_use('clam')
        style.configure('Dog.Treeview', background='#162331', foreground='#e4edf5',
                        fieldbackground='#162331', rowheight=26)
        style.configure('Dog.Treeview.Heading', background='#263849', foreground='white')
        style.map('Dog.Treeview', background=[('selected', '#284b62')])
        outer = tk.Frame(root, background='#101923', padx=24, pady=20)
        outer.pack(fill='both', expand=True)
        tk.Label(outer, text='DOG CONTROL', font=('Segoe UI', 23, 'bold'),
                 background='#101923', foreground='#eff6fc').pack(anchor='w')
        self.summary = tk.StringVar(value='正在连接…')
        tk.Label(outer, textvariable=self.summary, anchor='w', font=('Microsoft YaHei', 11),
                 background='#101923', foreground='#92afc3').pack(fill='x', pady=(3, 14))
        buttons = tk.Frame(outer, background='#101923')
        buttons.pack(fill='x')
        self.enable_button = self._button(buttons, '使能', 'enable', '#247960', 0)
        self.stand_button = self._button(buttons, 'STAND\n站立', 'stand', '#284c6c', 1)
        self.walk_button = self._button(buttons, 'WALK\n行走', 'walk', '#284c6c', 2)
        self.trot_button = self._button(buttons, 'TROT\n原地小跑', 'trot', '#284c6c', 3)
        emergency = tk.Frame(outer, background='#101923')
        emergency.pack(fill='x', pady=(14, 8))
        self.disable_button = tk.Button(emergency, text='失能 · 全部电机',
            command=lambda: self.command('disable'), font=('Microsoft YaHei', 18, 'bold'),
            background='#b52c3b', activebackground='#d03747', foreground='white',
            activeforeground='white', relief='flat', cursor='hand2', pady=14,
            takefocus=True)
        self.disable_button.pack(fill='x')
        tk.Label(outer, text='快捷失能：空格 / Esc    ·    WALK 和 TROT 要求狗已处于站姿',
                 background='#101923', foreground='#d1dce6', font=('Microsoft YaHei', 10)
                 ).pack(anchor='w', pady=(0, 10))
        self.message = tk.StringVar(value='正在等待 STM32 反馈')
        tk.Label(outer, textvariable=self.message, justify='left', anchor='w',
                 background='#1b2b3a', foreground='#f1c573', wraplength=860,
                 padx=12, pady=10, font=('Microsoft YaHei', 10)).pack(fill='x', pady=(0, 12))
        self.table = ttk.Treeview(outer, style='Dog.Treeview', show='headings', height=12,
            columns=('id', 'joint', 'state', 'q', 'dq', 'tau', 'temp'))
        for column, title, width in [('id', 'ID', 45), ('joint', '关节', 95),
                ('state', '状态', 100), ('q', '位置 rad', 120),
                ('dq', '速度 rad/s', 120), ('tau', '力矩 Nm', 120), ('temp', 'MOS / 线圈 °C', 130)]:
            self.table.heading(column, text=title)
            self.table.column(column, width=width, anchor='center')
        for index in range(12):
            self.table.insert('', 'end', iid=str(index+1), values=(index+1, '—', '等待', '—', '—', '—', '—'))
        self.table.pack(fill='x')
        self.log = tk.Text(outer, height=4, background='#101923', foreground='#98aebe',
                           relief='flat', font=('Microsoft YaHei', 9), state='disabled')
        self.log.pack(fill='both', expand=True, pady=(12, 0))
        for key in ('<Escape>', '<space>'):
            root.bind_all(key, self._emergency_key)
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.report_callback_exception = self._callback_error
        controller.start()
        self.refresh()

    def _button(self, parent, title, action, color, column):
        parent.columnconfigure(column, weight=1)
        button = tk.Button(parent, text=title, command=lambda: self.command(action),
            background=color, foreground='white', activebackground='#45647d',
            activeforeground='white', font=('Microsoft YaHei', 12, 'bold'),
            relief='flat', pady=10, cursor='hand2', disabledforeground='#8095a4')
        button.grid(row=0, column=column, sticky='nsew', padx=(0 if column == 0 else 8, 0))
        return button

    def _emergency_key(self, event=None):
        self.command('disable')
        return 'break'

    def command(self, action):
        try:
            self.controller.request(action)
        except Exception as error:
            self.controller._say(str(error))

    def refresh(self):
        status = self.controller.status()
        mode = (status['mode'] or ('失能中' if status['disabling'] else
                 '失能未确认' if status['disable_state'] == 'unconfirmed' else '未使能')).upper()
        stamp = status['feedback_age']
        age = f'{stamp*1000:.0f} ms' if stamp < 1000 else '无反馈'
        prefix = '离线演练' if status['dry_run'] else '真机'
        self.summary.set(f'{prefix}  ·  {status["port"] or "等待端口"}  ·  {mode}  ·  USB反馈 {age}')
        self.message.set(status['message'])
        can_enable = (status['connected'] and not status['enabled'] and not status['disabling']
                      and status['disable_state'] != 'unconfirmed' and not self.closing)
        self.enable_button.configure(state='normal' if can_enable else 'disabled')
        for button in (self.stand_button, self.walk_button, self.trot_button):
            button.configure(state='normal' if status['enabled'] and not self.closing else 'disabled')
        # The emergency button stays available regardless of action, feedback,
        # calibration, or connection state. No modal confirmations block it.
        joints = {r['can_id']: r['joint'] for r in self.controller.calibration
                  if isinstance(r.get('can_id'), int)}
        for row in status['feedback'] or []:
            state = ('使能' if row['state'] == 1 else '失能') if row['state'] in (0, 1) else f'故障 {row["state"]}'
            if not row['valid'] or stamp > self.controller.options['feedback_timeout']:
                state = '反馈无效/超时'
            self.table.item(str(row['id']), values=(row['id'], joints.get(row['id'], '未标定'),
                state, f'{row["position"]:+.4f}', f'{row["velocity"]:+.3f}',
                f'{row["torque"]:+.3f}', f'{row["tmos"]:.0f} / {row["tcoil"]:.0f}'))
        events = tuple(status['events'])
        if events != self._last_events:
            self.log.configure(state='normal')
            self.log.delete('1.0', 'end')
            self.log.insert('end', '\n'.join(events))
            self.log.see('end')
            self.log.configure(state='disabled')
            self._last_events = events
        self.root.after(50, self.refresh)

    def _callback_error(self, kind, error, traceback):
        self.controller.disable('UI 异常：'+str(error))

    def close(self):
        if self.closing:
            return
        self.closing = True
        self.controller.disable('关闭窗口，先全局失能')
        self.root.after(50, self._finish_close)

    def _finish_close(self):
        if self.controller.status()['disabling']:
            self.root.after(50, self._finish_close)
            return
        # Print the result as well: a failed stop must not disappear with the UI.
        print(self.controller.status()['message'], flush=True)
        self.controller.close()
        self.root.destroy()


def launch(initial_action=None):
    parser = argparse.ArgumentParser(description='机械狗桌面控制台；空格/Esc 一键失能')
    parser.add_argument('--dry-run', action='store_true', help='离线演练；不打开串口、不驱动电机')
    parser.add_argument('--config', help='配置 JSON，默认脚本旁 dog_config.json')
    parser.add_argument('--check', action='store_true', help='仅检查配置，不打开 UI 或串口')
    args = parser.parse_args()
    config, calibration = load_settings(args.config)
    controller = DogController(config, calibration, dry_run=args.dry_run, initial_action=initial_action)
    if args.check:
        import json
        problems = [] if args.dry_run else calibration_problems(config, calibration, controller.kin)
        print(json.dumps(dict(entry=initial_action or 'ui', dry_run=args.dry_run,
                             ready=not problems, pending=problems, serial_opened=False),
                         indent=2, ensure_ascii=False))
        return 0 if not problems else 2
    root = tk.Tk()
    app = DogUI(root, controller)
    signal.signal(signal.SIGINT, lambda *_: app.close())
    try:
        root.mainloop()
    finally:
        if not app.closing:
            controller.disable('UI 意外退出')
        controller.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(launch())

// rknputop
// Copyright (C) 2024 Amardeep Biswas
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

use std::collections::HashMap;
use std::env;
use std::fs;
use std::io;
use std::process::Command;
use std::time::{Duration, Instant};
use ratatui::widgets::ListState;

use crossterm::{
    event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::{
    backend::{Backend, CrosstermBackend},
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Gauge, List, ListItem, Paragraph},
    Frame, Terminal,
};
use regex::Regex;

struct Args {
    npu_only: bool,
    npu_bars: bool,
}

impl Args {
    fn parse() -> Self {
        let args: Vec<String> = env::args().collect();
        let mut npu_only = false;
        let mut npu_bars = false;

        for arg in args.iter() {
            match arg.as_str() {
                "-n" | "--npu-only" => npu_only = true,
                "-b" | "--npu-bars" => npu_bars = true,
                "-h" | "--help" => {
                    println!("rknputop - NPU/CPU/Memory/Thermal monitoring tool");
                    println!("Usage: rknputop [OPTIONS]");
                    println!("Options:");
                    println!("  -n, --npu-only    Only show the NPU load");
                    println!("  -b, --npu-bars    Show the NPU with bars instead of lines");
                    println!("  -h, --help        Show this help message");
                    std::process::exit(0);
                }
                _ => {}
            }
        }

        Self { npu_only, npu_bars }
    }
}

#[derive(Debug, Clone)]
struct ThermalInfo {
    label: String,
    current: f32,
    high: f32,
}

#[derive(Debug, Clone)]
struct CpuStats {
    user: u64,
    nice: u64,
    system: u64,
    idle: u64,
    iowait: u64,
    irq: u64,
    softirq: u64,
    steal: u64,
}

impl CpuStats {
    fn total(&self) -> u64 {
        self.user + self.nice + self.system + self.idle + self.iowait + self.irq + self.softirq + self.steal
    }

    fn active(&self) -> u64 {
        self.user + self.nice + self.system + self.irq + self.softirq + self.steal
    }
}

#[derive(Debug, Clone)]
struct LoadAverage {
    one_min: f32,
    five_min: f32,
    fifteen_min: f32,
    running_processes: u32,
    total_processes: u32,
}

#[derive(Debug, Clone)]
struct SystemStats {
    npu_loads: Vec<u32>,
    cpu_loads: Vec<f32>,
    memory_percent: f64,
    swap_percent: f64,
    thermals: Vec<ThermalInfo>,
    kernel_release: String,
    kernel_version: String,
    load_average: LoadAverage,
}

struct App {
    current_stats: SystemStats,
    args: Args,
    last_update: Instant,
    _prev_cpu_stats: HashMap<String, CpuStats>,
    npu_processes: Vec<(u32, String)>,
    selected_npu_process: Option<usize>,
}

impl App {
    fn new(args: Args) -> Result<Self, Box<dyn std::error::Error>> {
        let _prev_cpu_stats = read_cpu_stats_raw();
        let current_stats = read_system_stats(&_prev_cpu_stats)?;
        let npu_processes = get_npu_processes(); // Initial fetch

        Ok(Self {
            current_stats,
            args,
            last_update: Instant::now(),
            _prev_cpu_stats,
            npu_processes,
            selected_npu_process: None,
        })
    }

    fn update(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        if self.last_update.elapsed() >= Duration::from_millis(500) {
            // Update CPU stats
            let new_cpu_stats = read_cpu_stats_raw();
            let cpu_loads = calculate_cpu_usage(&self._prev_cpu_stats, &new_cpu_stats);
            self._prev_cpu_stats = new_cpu_stats;

            // Refresh all stats
            let npu_load_str = read_npu_load()?;
            let npu_loads = parse_npu_load(&npu_load_str);
            let (memory_percent, swap_percent) = read_memory_stats();
            let thermals = read_thermal_info();
            let kernel_release = read_kernel_release();
            let kernel_version = read_kernel_version();
            let load_average = read_load_average();
            let npu_processes = get_npu_processes(); // Refresh process list

            self.current_stats = SystemStats {
                npu_loads,
                cpu_loads,
                memory_percent,
                swap_percent,
                thermals,
                kernel_release,
                kernel_version,
                load_average,
            };

            self.npu_processes = npu_processes;

            // Clamp selection index if needed
            if let Some(selected) = self.selected_npu_process {
                if selected >= self.npu_processes.len() {
                    self.selected_npu_process = None;
                }
            }

            self.last_update = Instant::now();
        }
        Ok(())
    }
}


fn read_npu_load() -> Result<String, Box<dyn std::error::Error>> {
    match fs::read_to_string("/proc/rknpu/load") {
        Ok(content) => Ok(content),
        Err(_) => {
            eprintln!("Cannot read /proc/rknpu/load. Run with `sudo`");
            std::process::exit(-2);
        }
    }
}

fn read_kernel_release() -> String {
    match Command::new("uname").arg("-r").output() {
        Ok(output) => String::from_utf8_lossy(&output.stdout).trim().to_string(),
        Err(_) => "unknown".to_string(),
    }
}

fn read_kernel_version() -> String {
    match fs::read_to_string("/proc/version") {
        Ok(content) => content.trim().to_string(),
        Err(_) => "unknown".to_string(),
    }
}

fn read_load_average() -> LoadAverage {
    if let Ok(content) = fs::read_to_string("/proc/loadavg") {
        let parts: Vec<&str> = content.split_whitespace().collect();
        if parts.len() >= 5 {
            let one_min = parts[0].parse().unwrap_or(0.0);
            let five_min = parts[1].parse().unwrap_or(0.0);
            let fifteen_min = parts[2].parse().unwrap_or(0.0);
            
            if let Some(process_part) = parts.get(3) {
                let process_parts: Vec<&str> = process_part.split('/').collect();
                if process_parts.len() == 2 {
                    let running = process_parts[0].parse().unwrap_or(0);
                    let total = process_parts[1].parse().unwrap_or(0);
                    return LoadAverage {
                        one_min,
                        five_min,
                        fifteen_min,
                        running_processes: running,
                        total_processes: total,
                    };
                }
            }
        }
    }
    
    LoadAverage {
        one_min: 0.0,
        five_min: 0.0,
        fifteen_min: 0.0,
        running_processes: 0,
        total_processes: 0,
    }
}

fn parse_npu_load(txt: &str) -> Vec<u32> {
    let mut results = Vec::new();

    if txt.contains("Core0:") {
        let re = Regex::new(r"Core\d+:\s*(\d+)%").unwrap();
        for cap in re.captures_iter(txt) {
            if let Ok(pct) = cap[1].parse::<u32>() {
                results.push(pct);
            }
        }
    } else {
        let re = Regex::new(r"NPU load:\s+(\d+)%").unwrap();
        for cap in re.captures_iter(txt) {
            if let Ok(pct) = cap[1].parse::<u32>() {
                results.push(pct);
            }
        }
    }

    results
}

fn read_cpu_stats_raw() -> HashMap<String, CpuStats> {
    let mut cpu_stats = HashMap::new();
    
    if let Ok(content) = fs::read_to_string("/proc/stat") {
        for line in content.lines() {
            if line.starts_with("cpu") {
                let parts: Vec<&str> = line.split_whitespace().collect();
                if parts.len() >= 8 {
                    let cpu_name = parts[0].to_string();
                    let user: u64 = parts[1].parse().unwrap_or(0);
                    let nice: u64 = parts[2].parse().unwrap_or(0);
                    let system: u64 = parts[3].parse().unwrap_or(0);
                    let idle: u64 = parts[4].parse().unwrap_or(0);
                    let iowait: u64 = parts[5].parse().unwrap_or(0);
                    let irq: u64 = parts[6].parse().unwrap_or(0);
                    let softirq: u64 = parts[7].parse().unwrap_or(0);
                    let steal: u64 = parts.get(8).unwrap_or(&"0").parse().unwrap_or(0);
                    
                    cpu_stats.insert(cpu_name, CpuStats {
                        user, nice, system, idle, iowait, irq, softirq, steal
                    });
                }
            }
        }
    }
    
    cpu_stats
}

fn calculate_cpu_usage(prev_stats: &HashMap<String, CpuStats>, curr_stats: &HashMap<String, CpuStats>) -> Vec<f32> {
    let mut cpu_loads = Vec::new();
    
    // Skip the aggregate "cpu" entry and process individual cores
    for i in 0..2 {
        let cpu_name = format!("cpu{}", i);
        if let (Some(prev), Some(curr)) = (prev_stats.get(&cpu_name), curr_stats.get(&cpu_name)) {
            let prev_total = prev.total();
            let curr_total = curr.total();
            let prev_active = prev.active();
            let curr_active = curr.active();
            
            let total_diff = curr_total.saturating_sub(prev_total);
            let active_diff = curr_active.saturating_sub(prev_active);
            
            let usage = if total_diff > 0 {
                (active_diff as f32 / total_diff as f32) * 100.0
            } else {
                0.0
            };
            
            cpu_loads.push(usage.min(100.0).max(0.0));
        } else {
            break;
        }
    }
    
    cpu_loads
}

fn read_memory_stats() -> (f64, f64) {
    let mut mem_total = 0u64;
    let mut mem_available = 0u64;
    let mut swap_total = 0u64;
    let mut swap_free = 0u64;

    if let Ok(content) = fs::read_to_string("/proc/meminfo") {
        for line in content.lines() {
            if line.starts_with("MemTotal:") {
                mem_total = line.split_whitespace().nth(1).unwrap_or("0").parse().unwrap_or(0);
            } else if line.starts_with("MemAvailable:") {
                mem_available = line.split_whitespace().nth(1).unwrap_or("0").parse().unwrap_or(0);
            } else if line.starts_with("SwapTotal:") {
                swap_total = line.split_whitespace().nth(1).unwrap_or("0").parse().unwrap_or(0);
            } else if line.starts_with("SwapFree:") {
                swap_free = line.split_whitespace().nth(1).unwrap_or("0").parse().unwrap_or(0);
            }
        }
    }

    let mem_used = mem_total.saturating_sub(mem_available);
    let mem_percent = if mem_total > 0 { (mem_used as f64 / mem_total as f64) * 100.0 } else { 0.0 };
    
    let swap_used = swap_total.saturating_sub(swap_free);
    let swap_percent = if swap_total > 0 { (swap_used as f64 / swap_total as f64) * 100.0 } else { 0.0 };

    (mem_percent, swap_percent)
}

fn read_thermal_info() -> Vec<ThermalInfo> {
    let mut thermals = Vec::new();

    if let Ok(entries) = fs::read_dir("/sys/class/thermal/") {
        for entry in entries.flatten() {
            let path = entry.path();
            if let Some(name) = path.file_name().and_then(|n| n.to_str()) {
                if name.starts_with("thermal_zone") {
                    let type_path = path.join("type");
                    if let Ok(thermal_type) = fs::read_to_string(&type_path) {
                        let thermal_type = thermal_type.trim();
                        if thermal_type.starts_with("test") {
                            continue;
                        }

                        let temp_path = path.join("temp");
                        let trip_path = path.join("trip_point_0_temp");

                        if let (Ok(temp_str), Ok(trip_str)) = (
                            fs::read_to_string(&temp_path),
                            fs::read_to_string(&trip_path),
                        ) {
                            if let (Ok(temp), Ok(trip)) =
                                (temp_str.trim().parse::<i32>(), trip_str.trim().parse::<i32>())
                            {
                                thermals.push(ThermalInfo {
                                    label: thermal_type.replace("_thermal", "").to_string(),
                                    current: (temp as f32) / 1000.0,
                                    high: (trip as f32) / 1000.0,
                                });
                            }
                        }
                    }
                }
            }
        }
    }

    thermals
}

fn read_system_stats(_prev_cpu_stats: &HashMap<String, CpuStats>) -> Result<SystemStats, Box<dyn std::error::Error>> {
    let npu_load_str = read_npu_load()?;
    let npu_loads = parse_npu_load(&npu_load_str);
    let cpu_loads = Vec::new(); // Will be calculated in update()
    let (memory_percent, swap_percent) = read_memory_stats();
    let thermals = read_thermal_info();
    let kernel_release = read_kernel_release();
    let kernel_version = read_kernel_version();
    let load_average = read_load_average();

    Ok(SystemStats {
        npu_loads,
        cpu_loads,
        memory_percent,
        swap_percent,
        thermals,
        kernel_release,
        kernel_version,
        load_average,
    })
}

fn ui(f: &mut Frame, app: &App) {
    if app.args.npu_only {
        render_npu_only(f, app);
    } else {
        render_full_dashboard(f, app);
    }
}

fn render_npu_only(f: &mut Frame, app: &App) {
    let size = f.size();
    render_npu_htop_style(f, size, &app.current_stats.npu_loads);
}

fn render_full_dashboard(f: &mut Frame, app: &App) {
    let size = f.size();

    // Calculate line requirements for layout
    let npu_lines = if app.current_stats.npu_loads.is_empty() { 1 } else { app.current_stats.npu_loads.len() + 2 };
    let cpu_lines = if app.current_stats.cpu_loads.is_empty() { 1 } else { app.current_stats.cpu_loads.len() + 2 };
    let memory_lines = 4;
    let thermal_lines = app.current_stats.thermals.len().max(1) + 2;
    let info_lines = 7;
    let npu_proc_lines = app.npu_processes.len().max(1) + 6; // add buffer

    let total_lines = npu_lines + cpu_lines + memory_lines + thermal_lines + info_lines + npu_proc_lines;
    let available_lines = size.height as usize;

    let mut constraints = Vec::new();
    if available_lines >= total_lines {
        constraints.push(Constraint::Length(npu_lines as u16));
        constraints.push(Constraint::Length(cpu_lines as u16));
        constraints.push(Constraint::Length(memory_lines as u16));
        constraints.push(Constraint::Length(thermal_lines as u16));
        constraints.push(Constraint::Length(info_lines as u16));
        constraints.push(Constraint::Length(npu_proc_lines as u16));
    } else {
        // Fit layout to available space
        constraints.push(Constraint::Percentage(20)); // NPU
        constraints.push(Constraint::Percentage(20)); // CPU
        constraints.push(Constraint::Percentage(20)); // Memory
        constraints.push(Constraint::Percentage(15)); // Thermals
        constraints.push(Constraint::Percentage(15)); // Info
        constraints.push(Constraint::Percentage(15)); // NPU PIDs
    }

    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints(constraints)
        .split(size);

    // Render individual dashboard components
    render_npu_htop_style(f, chunks[0], &app.current_stats.npu_loads);
    render_cpu_htop_style(f, chunks[1], &app.current_stats.cpu_loads);
    render_memory_htop_style(f, chunks[2], app.current_stats.memory_percent, app.current_stats.swap_percent);
    render_thermals_htop_style(f, chunks[3], &app.current_stats.thermals);
    render_npu_processes(f, chunks[4], &app.npu_processes, app.selected_npu_process);
    render_system_info(f, chunks[5], &app.current_stats);
}

fn render_npu_processes(
    f: &mut Frame,
    area: Rect,
    processes: &Vec<(u32, String)>,
    selected: Option<usize>,
 ) {
    let title = "NPU Processes";

    // Create process items
    let mut items: Vec<ListItem> = processes
        .iter()
        .map(|(pid, name)| ListItem::new(format!("{} - {}", pid, name)))
        .collect();

    let list = List::new(items)
        .block(Block::default().borders(Borders::ALL).title(title))
        .highlight_style(Style::default().bg(Color::Blue).fg(Color::White))
        .highlight_symbol(">> ");

    let mut state = ListState::default();
    state.select(selected);

    f.render_stateful_widget(list, area, &mut state);
}

fn render_npu_htop_style(f: &mut Frame, area: Rect, npu_loads: &[u32]) {
    let mut items = Vec::new();
    
    for (i, &load) in npu_loads.iter().enumerate() {
        let bar_width = ((area.width - 15) as f32 * (load as f32 / 100.0)) as usize;
        let bar = "█".repeat(bar_width);
        let spaces = " ".repeat((area.width - 15) as usize - bar_width);
        
        let color = match load {
            0..=50 => Color::Green,
            51..=80 => Color::Yellow,
            _ => Color::Red,
        };
        
        items.push(ListItem::new(Line::from(vec![
            Span::raw(format!("NPU{}: ", i)),
            Span::styled(format!("{:3}%", load), Style::default().fg(Color::White).add_modifier(Modifier::BOLD)),
            Span::raw(" ["),
            Span::styled(bar, Style::default().fg(color)),
            Span::raw(format!("{}]", spaces)),
        ])));
    }
    
    if items.is_empty() {
        items.push(ListItem::new("No NPU data available"));
    }

    let list = List::new(items)
        .block(Block::default().title("NPU Usage").borders(Borders::ALL));

    f.render_widget(list, area);
}

fn render_cpu_htop_style(f: &mut Frame, area: Rect, cpu_loads: &[f32]) {
    let mut items = Vec::new();
    
    for (i, &load) in cpu_loads.iter().enumerate() {
        let load_int = load.round() as u32;
        let bar_width = ((area.width - 15) as f32 * (load / 100.0)) as usize;
        let bar = "█".repeat(bar_width);
        let spaces = " ".repeat((area.width - 15) as usize - bar_width);
        
        let color = match load_int {
            0..=50 => Color::Green,
            51..=80 => Color::Yellow,
            _ => Color::Red,
        };
        
        items.push(ListItem::new(Line::from(vec![
            Span::raw(format!("CPU{}: ", i)),
            Span::styled(format!("{:3}%", load_int), Style::default().fg(Color::White).add_modifier(Modifier::BOLD)),
            Span::raw(" ["),
            Span::styled(bar, Style::default().fg(color)),
            Span::raw(format!("{}]", spaces)),
        ])));
    }
    
    if items.is_empty() {
        items.push(ListItem::new("CPU data loading..."));
    }

    let list = List::new(items)
        .block(Block::default().title("CPU Usage").borders(Borders::ALL));

    f.render_widget(list, area);
}

fn render_memory_htop_style(f: &mut Frame, area: Rect, mem_pct: f64, swap_pct: f64) {
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Length(1), Constraint::Length(1)])
        .split(Block::default().title("Memory").borders(Borders::ALL).inner(area));

    // Memory bar
    let mem_gauge = Gauge::default()
        .percent(mem_pct as u16)
        .label(format!("Mem: {:.1}%", mem_pct))
        .gauge_style(Style::default().fg(
            if mem_pct > 80.0 { Color::Red } 
            else if mem_pct > 60.0 { Color::Yellow } 
            else { Color::Green }
        ));

    // Swap bar
    let swap_gauge = Gauge::default()
        .percent(swap_pct as u16)
        .label(format!("Swp: {:.1}%", swap_pct))
        .gauge_style(Style::default().fg(
            if swap_pct > 50.0 { Color::Red } 
            else if swap_pct > 20.0 { Color::Yellow } 
            else { Color::Blue }
        ));

    f.render_widget(
        Block::default().title("Memory").borders(Borders::ALL),
        area,
    );
    
    if chunks.len() >= 2 {
        f.render_widget(mem_gauge, chunks[0]);
        f.render_widget(swap_gauge, chunks[1]);
    }
}

fn render_thermals_htop_style(f: &mut Frame, area: Rect, thermals: &[ThermalInfo]) {
    let mut items = Vec::new();
    
    for thermal in thermals {
        let temp_pct = ((thermal.current / thermal.high) * 100.0) as u16;
        let color = if temp_pct > 80 { Color::Red } else if temp_pct > 60 { Color::Yellow } else { Color::Green };
        
        items.push(ListItem::new(Line::from(vec![
            Span::raw(format!("{}: ", thermal.label)),
            Span::styled(
                format!("{:.1}°C ({:.0}%)", thermal.current, temp_pct),
                Style::default().fg(color).add_modifier(Modifier::BOLD),
            ),
        ])));
    }
    
    if items.is_empty() {
        items.push(ListItem::new("No thermal sensors found"));
    }

    let list = List::new(items)
        .block(Block::default().title("Thermals").borders(Borders::ALL));

    f.render_widget(list, area);
}

fn render_system_info(f: &mut Frame, area: Rect, stats: &SystemStats) {
    let text = vec![
        Line::from(vec![
            Span::raw("Load avg: "),
            Span::styled(
                format!("{:.2} {:.2} {:.2}", 
                    stats.load_average.one_min, 
                    stats.load_average.five_min, 
                    stats.load_average.fifteen_min
                ),
                Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD),
            ),
            Span::raw(format!(" Tasks: {}/{}", 
                stats.load_average.running_processes, 
                stats.load_average.total_processes
            )),
        ]),
        Line::from(vec![
            Span::raw("Kernel: "),
            Span::styled(&stats.kernel_release, Style::default().fg(Color::Green)),
        ]),
        Line::from(vec![
            Span::styled("Press 'q' to quit", Style::default().fg(Color::Yellow)),
        ]),
        Line::from(vec![
            Span::styled("Press '↑ or ↓' to navigate the processes ", Style::default().fg(Color::Yellow)),
        ]),
        Line::from(vec![
            Span::styled("Press 'k' to kill process", Style::default().fg(Color::Red)),
        ]),
    ];

    let paragraph = Paragraph::new(text)
        .block(Block::default().title("System Info").borders(Borders::ALL));
    f.render_widget(paragraph, area);
}

fn get_npu_processes() -> Vec<(u32, String)> {
    let mut npu_procs = Vec::new();
    let npu_keywords = ["rknn", "rknpu"];

    if let Ok(entries) = fs::read_dir("/proc") {
        for entry in entries.flatten() {
            if let Ok(pid) = entry.file_name().to_string_lossy().parse::<u32>() {
                let maps_path = format!("/proc/{}/maps", pid);
                let cmdline_path = format!("/proc/{}/cmdline", pid);

                if let Ok(maps) = fs::read_to_string(&maps_path) {
                    if npu_keywords.iter().any(|kw| maps.contains(kw)) {
                        let cmdline = fs::read_to_string(&cmdline_path)
                            .unwrap_or_default()
                            .replace('\0', " ");
                        npu_procs.push((pid, cmdline.trim().to_string()));
                    }
                }
            }
        }
    }

    npu_procs
}


fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    let initial_load = read_npu_load()?;
    if initial_load.trim().is_empty() {
        eprintln!("Cannot read anything in /proc/rknpu/load. Run with `sudo`");
        std::process::exit(-2);
    }

    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let mut app = App::new(args)?;
    let res = run_app(&mut terminal, &mut app);

    disable_raw_mode()?;
    execute!(
        terminal.backend_mut(),
        LeaveAlternateScreen,
        DisableMouseCapture
    )?;
    terminal.show_cursor()?;

    if let Err(err) = res {
        println!("{:?}", err);
    }

    Ok(())
}

fn run_app<B: Backend>(
    terminal: &mut Terminal<B>,
    app: &mut App,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        app.update()?;
        terminal.draw(|f| ui(f, app))?;

        if event::poll(Duration::from_millis(100))? {
            if let Event::Key(key) = event::read()? {
                match key.code {
                    KeyCode::Char('q') | KeyCode::Char('Q') | KeyCode::Esc => {
                        return Ok(());
                    }
                    KeyCode::Up => {
                        if let Some(i) = app.selected_npu_process {
                            if i > 0 {
                                app.selected_npu_process = Some(i - 1);
                            }
                        } else if !app.npu_processes.is_empty() {
                            app.selected_npu_process = Some(0);
                        }
                    }
                    KeyCode::Down => {
                        if let Some(i) = app.selected_npu_process {
                            if i + 1 < app.npu_processes.len() {
                                app.selected_npu_process = Some(i + 1);
                            }
                        } else if !app.npu_processes.is_empty() {
                            app.selected_npu_process = Some(0);
                        }
                    }
                    KeyCode::Char('k') => {
                        if let Some(i) = app.selected_npu_process {
                            if let Some((pid, _)) = app.npu_processes.get(i) {
                                use nix::sys::signal::{kill, Signal};
                                use nix::unistd::Pid;
                                let _ = kill(Pid::from_raw(*pid as i32), Signal::SIGKILL);
                            }
                        }
                    }
                    _ => {}
                }
            }
        }
    }
}

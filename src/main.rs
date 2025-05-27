// rknputop
// Copyright (C) 2024 Broox Technologies Ltd.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

use std::collections::VecDeque;
use std::env;
use std::fs;
use std::io;
use std::time::{Duration, Instant};

use crossterm::{
    event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::{
    backend::{Backend, CrosstermBackend},
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Style},
    symbols,
    text::{Line, Span},
    widgets::{Axis, BarChart, Block, Borders, Chart, Dataset, Gauge, List, ListItem, Paragraph},
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
struct SystemStats {
    npu_loads: Vec<u32>,
    cpu_loads: Vec<f32>,
    memory_percent: f64,
    swap_percent: f64,
    thermals: Vec<ThermalInfo>,
    kernel_version: String,
    lib_version: String,
}

const MAX_SAMPLES: usize = 100;

struct App {
    npu_history: VecDeque<Vec<u32>>,
    cpu_history: VecDeque<Vec<f32>>,
    current_stats: SystemStats,
    args: Args,
    last_update: Instant,
}

impl App {
    fn new(args: Args) -> Result<Self, Box<dyn std::error::Error>> {
        let current_stats = read_system_stats()?;
        
        Ok(Self {
            npu_history: VecDeque::with_capacity(MAX_SAMPLES),
            cpu_history: VecDeque::with_capacity(MAX_SAMPLES),
            current_stats,
            args,
            last_update: Instant::now(),
        })
    }

    fn update(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        if self.last_update.elapsed() >= Duration::from_secs(1) {
            self.current_stats = read_system_stats()?;
            
            self.npu_history.push_back(self.current_stats.npu_loads.clone());
            if self.npu_history.len() > MAX_SAMPLES {
                self.npu_history.pop_front();
            }

            self.cpu_history.push_back(self.current_stats.cpu_loads.clone());
            if self.cpu_history.len() > MAX_SAMPLES {
                self.cpu_history.pop_front();
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

fn read_kernel_version() -> String {
    match fs::read_to_string("/sys/module/rknpu/version") {
        Ok(content) => {
            let parts: Vec<&str> = content.split(':').collect();
            if parts.len() > 1 {
                parts[1].trim().to_string()
            } else {
                "unknown".to_string()
            }
        }
        Err(_) => "unknown".to_string(),
    }
}

fn read_lib_version() -> String {
    "unknown".to_string()
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

fn read_cpu_stats() -> Vec<f32> {
    match fs::read_to_string("/proc/stat") {
        Ok(content) => {
            let mut cpu_loads = Vec::new();
            for line in content.lines() {
                if line.starts_with("cpu") && line != "cpu" {
                    let parts: Vec<&str> = line.split_whitespace().collect();
                    if parts.len() >= 5 {
                        let user: u64 = parts[1].parse().unwrap_or(0);
                        let nice: u64 = parts[2].parse().unwrap_or(0);
                        let system: u64 = parts[3].parse().unwrap_or(0);
                        let idle: u64 = parts[4].parse().unwrap_or(0);
                        let total = user + nice + system + idle;
                        let used = user + nice + system;
                        let usage = if total > 0 { (used as f32 / total as f32) * 100.0 } else { 0.0 };
                        cpu_loads.push(usage);
                    }
                }
            }
            cpu_loads
        }
        Err(_) => Vec::new(),
    }
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

fn read_system_stats() -> Result<SystemStats, Box<dyn std::error::Error>> {
    let npu_load_str = read_npu_load()?;
    let npu_loads = parse_npu_load(&npu_load_str);
    let cpu_loads = read_cpu_stats();
    let (memory_percent, swap_percent) = read_memory_stats();
    let thermals = read_thermal_info();
    let kernel_version = read_kernel_version();
    let lib_version = read_lib_version();

    Ok(SystemStats {
        npu_loads,
        cpu_loads,
        memory_percent,
        swap_percent,
        thermals,
        kernel_version,
        lib_version,
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
    
    if app.args.npu_bars {
        render_npu_bars(f, size, &app.current_stats.npu_loads);
    } else {
        render_npu_chart(f, size, &app.npu_history);
    }
}

fn render_full_dashboard(f: &mut Frame, app: &App) {
    let size = f.size();
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Percentage(40),
            Constraint::Percentage(30),
            Constraint::Percentage(30),
        ])
        .split(size);

    let top_chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(chunks[0]);

    // NPU
    if app.args.npu_bars {
        render_npu_bars(f, top_chunks[0], &app.current_stats.npu_loads);
    } else {
        render_npu_chart(f, top_chunks[0], &app.npu_history);
    }

    // CPU
    render_cpu_bars(f, top_chunks[1], &app.current_stats.cpu_loads);

    let bottom_chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(chunks[1]);

    // Memory
    render_memory_gauges(f, bottom_chunks[0], app.current_stats.memory_percent, app.current_stats.swap_percent);

    // Thermals
    render_thermals(f, bottom_chunks[1], &app.current_stats.thermals);

    // Version info
    render_version_info(f, chunks[2], &app.current_stats.kernel_version, &app.current_stats.lib_version);
}

fn render_npu_bars(f: &mut Frame, area: Rect, npu_loads: &[u32]) {
    let data: Vec<(String, u64)> = npu_loads
        .iter()
        .enumerate()
        .map(|(i, &load)| (format!("C{}", i), load as u64))
        .collect();

    let data_refs: Vec<(&str, u64)> = data.iter().map(|(s, v)| (s.as_str(), *v)).collect();

    let barchart = BarChart::default()
        .block(Block::default().title("NPU Load").borders(Borders::ALL))
        .data(&data_refs)
        .bar_width(3)
        .bar_style(Style::default().fg(Color::Yellow))
        .value_style(Style::default().fg(Color::Black).bg(Color::Yellow));

    f.render_widget(barchart, area);
}

fn render_npu_chart(f: &mut Frame, area: Rect, npu_history: &VecDeque<Vec<u32>>) {
    if npu_history.is_empty() {
        return;
    }

    let num_cores = npu_history[0].len();
    let mut all_data: Vec<Vec<(f64, f64)>> = Vec::new();
    let mut datasets = Vec::new();

    for core in 0..num_cores {
        let data: Vec<(f64, f64)> = npu_history
            .iter()
            .enumerate()
            .map(|(i, loads)| (i as f64, *loads.get(core).unwrap_or(&0) as f64))
            .collect();
        all_data.push(data);
    }

    for (core, data) in all_data.iter().enumerate() {
        let color = match core {
            0 => Color::Red,
            1 => Color::Green,
            2 => Color::Blue,
            3 => Color::Yellow,
            _ => Color::White,
        };

        datasets.push(
            Dataset::default()
                .name(format!("Core {}", core))
                .marker(symbols::Marker::Braille)
                .style(Style::default().fg(color))
                .data(data),
        );
    }

    let chart = Chart::new(datasets)
        .block(Block::default().title("NPU Load History").borders(Borders::ALL))
        .x_axis(Axis::default().title("Time").bounds([0.0, MAX_SAMPLES as f64]))
        .y_axis(Axis::default().title("Load %").bounds([0.0, 100.0]));

    f.render_widget(chart, area);
}

fn render_cpu_bars(f: &mut Frame, area: Rect, cpu_loads: &[f32]) {
    let data: Vec<(String, u64)> = cpu_loads
        .iter()
        .enumerate()
        .map(|(i, &load)| (format!("C{}", i), load as u64))
        .collect();

    let data_refs: Vec<(&str, u64)> = data.iter().map(|(s, v)| (s.as_str(), *v)).collect();

    let barchart = BarChart::default()
        .block(Block::default().title("CPU Load").borders(Borders::ALL))
        .data(&data_refs)
        .bar_width(2)
        .bar_style(Style::default().fg(Color::Green))
        .value_style(Style::default().fg(Color::Black).bg(Color::Green));

    f.render_widget(barchart, area);
}

fn render_memory_gauges(f: &mut Frame, area: Rect, mem_pct: f64, swap_pct: f64) {
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(area);

    let mem_gauge = Gauge::default()
        .block(Block::default().title("Memory").borders(Borders::ALL))
        .gauge_style(Style::default().fg(Color::Blue))
        .percent(mem_pct as u16)
        .label(format!("{:.1}%", mem_pct));

    let swap_gauge = Gauge::default()
        .block(Block::default().title("Swap").borders(Borders::ALL))
        .gauge_style(Style::default().fg(Color::Magenta))
        .percent(swap_pct as u16)
        .label(format!("{:.1}%", swap_pct));

    f.render_widget(mem_gauge, chunks[0]);
    f.render_widget(swap_gauge, chunks[1]);
}

fn render_thermals(f: &mut Frame, area: Rect, thermals: &[ThermalInfo]) {
    let items: Vec<ListItem> = thermals
        .iter()
        .map(|thermal| {
            let pct = (thermal.current / thermal.high * 100.0) as u16;
            ListItem::new(Line::from(vec![
                Span::raw(format!("{}: ", thermal.label)),
                Span::styled(
                    format!("{:.1}°C ({}%)", thermal.current, pct),
                    Style::default().fg(if pct > 80 { Color::Red } else { Color::Green }),
                ),
            ]))
        })
        .collect();

    let list = List::new(items)
        .block(Block::default().title("Thermals").borders(Borders::ALL));

    f.render_widget(list, area);
}

fn render_version_info(f: &mut Frame, area: Rect, kernel_ver: &str, lib_ver: &str) {
    let text = vec![
        Line::from(vec![
            Span::raw("Kernel: "),
            Span::styled(kernel_ver, Style::default().fg(Color::Cyan)),
        ]),
        Line::from(vec![
            Span::raw("Library: "),
            Span::styled(lib_ver, Style::default().fg(Color::Cyan)),
        ]),
        Line::from(Span::styled(
            "Press 'q' to quit",
            Style::default().fg(Color::Yellow),
        )),
    ];

    let paragraph = Paragraph::new(text)
        .block(Block::default().title("Info").borders(Borders::ALL));

    f.render_widget(paragraph, area);
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    let initial_load = read_npu_load()?;
    if initial_load.trim().is_empty() {
        eprintln!("Cannot read anything in /sys/kernel/debug/rknpu/load. Run with `sudo`");
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
                    _ => {}
                }
            }
        }
    }
}
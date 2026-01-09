#!/usr/bin/env python3
"""
Mission Log Analyzer

Analyzes mission logs for debugging and performance analysis.
Usage: ros2 run tyre_inspection_mission analyze_mission_log <log_file>
"""

import json
import sys
from pathlib import Path
from typing import Dict, List, Any
from collections import defaultdict
from datetime import timedelta


def load_log_file(log_file: Path) -> List[Dict]:
    """Load mission log file (JSONL format)"""
    events = []
    with open(log_file, 'r') as f:
        for line in f:
            if line.strip():
                try:
                    events.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
    return events


def analyze_mission(events: List[Dict]) -> Dict[str, Any]:
    """Analyze mission events"""
    analysis = {
        'total_events': len(events),
        'duration': 0.0,
        'events_by_type': defaultdict(int),
        'state_transitions': [],
        'errors': [],
        'issues': [],
        'detections': defaultdict(int),
        'navigations': [],
        'captures': {'success': 0, 'failed': 0},
        'statistics': {}
    }
    
    if not events:
        return analysis
    
    # Calculate duration
    if events:
        first_event = events[0]
        last_event = events[-1]
        analysis['duration'] = last_event.get('elapsed', 0.0)
    
    # Analyze events
    for event in events:
        event_type = event.get('event_type', 'unknown')
        analysis['events_by_type'][event_type] += 1
        
        data = event.get('data', {})
        
        if event_type == 'state_transition':
            analysis['state_transitions'].append({
                'from': data.get('from'),
                'to': data.get('to'),
                'time': event.get('elapsed', 0.0),
                'reason': data.get('reason')
            })
        
        elif event_type == 'error':
            analysis['errors'].append({
                'type': data.get('error_type'),
                'code': data.get('error_code'),
                'message': data.get('message'),
                'time': event.get('elapsed', 0.0)
            })
        
        elif event_type == 'issue':
            analysis['issues'].append({
                'type': data.get('issue_type'),
                'severity': data.get('severity'),
                'recommendation': data.get('recommendation'),
                'time': event.get('elapsed', 0.0)
            })
        
        elif event_type == 'detection':
            obj_type = data.get('object_type', 'unknown')
            analysis['detections'][obj_type] += 1
        
        elif event_type == 'navigation':
            analysis['navigations'].append({
                'action': data.get('action'),
                'target': data.get('target'),
                'result': data.get('result'),
                'time': event.get('elapsed', 0.0)
            })
        
        elif event_type == 'capture':
            if data.get('success'):
                analysis['captures']['success'] += 1
            else:
                analysis['captures']['failed'] += 1
    
    # Calculate statistics
    analysis['statistics'] = {
        'error_rate': len(analysis['errors']) / (analysis['duration'] / 60.0) if analysis['duration'] > 0 else 0,
        'issue_count': len(analysis['issues']),
        'critical_issues': sum(1 for i in analysis['issues'] if i['severity'] == 'high'),
        'state_count': len(set(t['to'] for t in analysis['state_transitions'])),
        'capture_success_rate': (
            analysis['captures']['success'] / 
            (analysis['captures']['success'] + analysis['captures']['failed'])
            if (analysis['captures']['success'] + analysis['captures']['failed']) > 0 else 0
        )
    }
    
    return analysis


def print_analysis(analysis: Dict[str, Any]):
    """Print analysis results"""
    print("\n" + "="*70)
    print("MISSION LOG ANALYSIS")
    print("="*70)
    
    # Summary
    print(f"\n📊 Summary:")
    print(f"  Total Events: {analysis['total_events']}")
    print(f"  Duration: {timedelta(seconds=int(analysis['duration']))}")
    print(f"  States Visited: {analysis['statistics']['state_count']}")
    
    # Events by type
    print(f"\n📋 Events by Type:")
    for event_type, count in sorted(analysis['events_by_type'].items()):
        print(f"  {event_type}: {count}")
    
    # Detections
    if analysis['detections']:
        print(f"\n🎯 Detections:")
        for obj_type, count in sorted(analysis['detections'].items()):
            print(f"  {obj_type}: {count}")
    
    # Captures
    if analysis['captures']['success'] + analysis['captures']['failed'] > 0:
        print(f"\n📸 Captures:")
        print(f"  Success: {analysis['captures']['success']}")
        print(f"  Failed: {analysis['captures']['failed']}")
        print(f"  Success Rate: {analysis['statistics']['capture_success_rate']*100:.1f}%")
    
    # Errors
    if analysis['errors']:
        print(f"\n❌ Errors ({len(analysis['errors'])}):")
        for error in analysis['errors'][:10]:  # Show first 10
            print(f"  [{error['time']:.1f}s] {error['code']}: {error['message']}")
        if len(analysis['errors']) > 10:
            print(f"  ... and {len(analysis['errors']) - 10} more")
    
    # Issues
    if analysis['issues']:
        print(f"\n⚠️  Issues ({len(analysis['issues'])}):")
        critical = [i for i in analysis['issues'] if i['severity'] == 'high']
        if critical:
            print(f"  Critical: {len(critical)}")
            for issue in critical[:5]:
                print(f"    [{issue['time']:.1f}s] {issue['type']}: {issue['recommendation']}")
        
        warnings = [i for i in analysis['issues'] if i['severity'] == 'medium']
        if warnings:
            print(f"  Warnings: {len(warnings)}")
    
    # State transitions
    if analysis['state_transitions']:
        print(f"\n🔄 State Transitions ({len(analysis['state_transitions'])}):")
        for trans in analysis['state_transitions'][-10:]:  # Last 10
            print(f"  [{trans['time']:.1f}s] {trans['from']} → {trans['to']}")
    
    # Statistics
    print(f"\n📈 Statistics:")
    print(f"  Error Rate: {analysis['statistics']['error_rate']:.2f} errors/min")
    print(f"  Critical Issues: {analysis['statistics']['critical_issues']}")
    
    print("\n" + "="*70)


def main():
    if len(sys.argv) < 2:
        print("Usage: analyze_mission_log <log_file>")
        print("\nExample:")
        print("  ros2 run tyre_inspection_mission analyze_mission_log ~/tyre_inspection_logs/mission_log_20240109_120000.jsonl")
        sys.exit(1)
    
    log_file = Path(sys.argv[1])
    
    if not log_file.exists():
        print(f"Error: Log file not found: {log_file}")
        sys.exit(1)
    
    print(f"Loading log file: {log_file}")
    events = load_log_file(log_file)
    
    if not events:
        print("Error: No events found in log file")
        sys.exit(1)
    
    print(f"Loaded {len(events)} events")
    analysis = analyze_mission(events)
    print_analysis(analysis)


if __name__ == '__main__':
    main()

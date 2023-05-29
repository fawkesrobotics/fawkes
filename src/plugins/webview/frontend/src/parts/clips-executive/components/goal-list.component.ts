
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Component, OnInit, OnDestroy, ViewChildren, AfterViewInit, QueryList } from '@angular/core';
import { Router } from '@angular/router';
import { MatTableDataSource } from '@angular/material/table';
import { DomSanitizer, SafeUrl } from '@angular/platform-browser';

import { BackendConfigurationService } from '../../../services/backend-config/backend-config.service';
import { ClipsExecutiveApiService } from '../services/api.service';
import { DotGraphComponent } from '../../../components/dotgraph/component';
import { Goal } from '../models/Goal';
import { Plan } from '../models/Plan';
import { DomainOperator } from '../models/DomainOperator';

import { interval, forkJoin } from 'rxjs';
import { GroundedFormula } from '../models/GroundedFormula';

@Component({
  selector: 'ff-clips-executive-goal-list',
  templateUrl: './goal-list.component.html',
  styleUrls: ['./goal-list.component.scss']
})
export class GoalListComponent implements OnInit, OnDestroy, AfterViewInit {

  @ViewChildren('dotgraph')
  public dotgraphs: QueryList<DotGraphComponent>;
  public dotgraph: DotGraphComponent;

  private backend_subscription = null;

  dataSource = new MatTableDataSource();
  displayedColumns = ['mode', 'id', 'type', 'class'];

  data_received = false;
  auto_refresh_subscription = null;

  plans = [];
  operators: DomainOperator[] = null;
  goals: Goal[] = null;
  goals_graph: string = null;
  loading = false;

  graph_svg_available = false;
  graph_svg_base64: SafeUrl = null;

  zero_message = 'No goals received.';

  constructor(private readonly api_service: ClipsExecutiveApiService,
              private router: Router,
              private backendcfg: BackendConfigurationService,
              private sanitizer: DomSanitizer) {}

  ngOnInit() {
    this.refresh_domain();
    this.refresh();
    this.backend_subscription = this.backendcfg.backend_changed.subscribe((b) => { this.refresh(); });
    this.graph_svg_base64 = this.sanitizer.bypassSecurityTrustUrl('#');
  }

  ngOnDestroy() {
    this.backend_subscription.unsubscribe();
    this.backend_subscription = null;
    this.disable_autorefresh();
  }

  ngAfterViewInit() {
    this.dotgraphs.changes.subscribe((comps: QueryList<DotGraphComponent>) => {
      this.dotgraph = comps.first;
      this.graph_svg_available = false;
      if (this.dotgraph) {
        this.dotgraph.svg_updated.subscribe((svg: string) => this.svg_updated(svg));
      }
    });
  }

  refresh() {
    this.loading = true;
    this.zero_message = 'Retrieving goals';

    this.api_service.list_goals().subscribe(
      (goals) => {
        // console.log("Goals", goals)
        const plans = [];
        goals
          .filter(g => g.plans.length > 0)
          .forEach(g => g.plans.forEach(p => plans.push({goal_id: g.id, plan_id: p})));

        this.dataSource.data = this.process_tree(goals);
        if (this.dataSource.data.length === 0) {
          this.zero_message = 'Executive has currently no goals';
        }

        this.goals = goals;
        this.plans = plans;

        if (this.plans.length > 0) {
          this.refresh_plans();
        } else {
          this.create_goals_graph();
          this.loading = false;
        }
      },
      (err) => {
        this.dataSource.data = [];
        if (err.status === 0) {
          this.zero_message = 'API server unavailable. Robot down?';
        } else {
          this.zero_message = `Failed to retrieve goals: ${err.error}`;
        }
        this.loading = false;
      }
    );
  }

  refresh_domain() {
    this.api_service.list_domain_operators()
      .subscribe(
        (operators) => { this.operators = operators; },
        (err) => { console.log('Failed to receive domain data'); }
      );
  }

  refresh_plans() {
    this.loading = true;
    this.zero_message = 'Retrieving plans';

    forkJoin(
      // forkJoin: retrieve all plans associated to goal in parallel
      this.plans.map((plan) => {
        return this.api_service.get_plan(plan.goal_id, plan.plan_id);
      })
    )
    .subscribe(
      (plans: Plan[]) => {
        for (let i = 0; i < plans.length; ++i) {
          // console.log("Got plan", this.plans[i].goal_id, this.plans[i].plan_id)
          this.plans[i].plan = plans[i];
        }
        // console.log("Plans", this.plans)
        this.create_goals_graph();
        this.loading = false;
      },
      (err) => {
        console.log('Failed to retrieve plans');
        this.create_goals_graph();
        this.loading = false;
      }
    );
  }

  recursive_add_goals(l, level: number,
                      goals: Goal[], sub_goals: Map<string, Goal[]>) {
    for (const g of goals) {
      l.push({goal: g, level: level, width: 16 * level});
      if (g.id in sub_goals) {
        this.recursive_add_goals(l, level + 1, sub_goals[g.id], sub_goals);
      }
    }
  }

  process_tree(data: Goal[]) {
    const rv = [];

    const top_goals: Goal[] = [];
    const sub_goals: Map<string, Goal[]> = new Map();
    for (const g of data) {
      if (! g.parent || g.parent === '') {
        top_goals.push(g);
      } else if (g.parent) {
        if (g.parent in sub_goals) {
          sub_goals[g.parent].push(g);
        } else {
          sub_goals[g.parent] = [g];
        }
      }
    }

    this.recursive_add_goals(rv, 0, top_goals, sub_goals);

    return rv;
  }

  icon_name(goal: Goal): string {
    switch (goal.mode) {
      case 'FORMULATED': return 'note_add';
      case 'SELECTED':   return 'build';
      case 'EXPANDED':   return 'assignment';
      case 'COMMITTED':   return 'assignment_turned_in';
      case 'DISPATCHED':   return 'loop';
      case 'FINISHED':
      {
        switch (goal.outcome) {
          case 'COMPLETED': return 'check';
          case 'FAILED': return 'error_outline';
          case 'REJECTED': return 'report';
          default: return 'help';
        }
      }
      case 'EVALUATED':
      {
        switch (goal.outcome) {
          case 'COMPLETED': return 'check_circle';
          case 'FAILED': return 'error';
          case 'REJECTED': return 'report';
          default: return 'help';
        }
      }
      case 'RETRACTED':   return 'delete';
      default: return 'help_outline';
    }
  }

  icon_class(goal: Goal): string {
    switch (goal.mode) {
      case 'FORMULATED': return 'ff-muted';
      case 'SELECTED':   return 'ff-primary';
      case 'EXPANDED':   return 'ff-primary';
      case 'COMMITTED':   return 'ff-primary';
      case 'DISPATCHED':   return 'ff-primary';
      case 'FINISHED':
      {
        switch (goal.outcome) {
          case 'COMPLETED': return 'ff-success';
          case 'FAILED': return 'ff-error';
          case 'REJECTED': return 'ff-warning';
          default: return 'ff-warning';
        }
      }
      case 'EVALUATED':
      {
        switch (goal.outcome) {
          case 'COMPLETED': return 'ff-success';
          case 'FAILED': return 'ff-error';
          case 'REJECTED': return 'ff-warning';
          default: return 'ff-warning';
        }
      }
      case 'RETRACTED':
        switch (goal.outcome) {
          case 'COMPLETED': return 'ff-success';
          case 'FAILED': return 'ff-error';
          case 'REJECTED': return 'ff-warning';
          default: return 'ff-primary';
        }
      default: return 'ff-warning';
    }
  }

  icon_tooltip(goal: Goal): string {
    switch (goal.mode) {
      case 'FINISHED':
      case 'EVALUATED':
        return `${goal.mode}|${goal.outcome}`;
      default:
        return goal.mode;
    }
  }

  goto_goal(goal: Goal) {
    this.router.navigate(['/clips-executive/goal/', goal.id]);
  }

  show_zero_state(): boolean {
    return ! this.dataSource.data || this.dataSource.data.length === 0;
  }

  private enable_autorefresh() {
    if (this.auto_refresh_subscription) {  return; }
    this.auto_refresh_subscription =
      interval(2000).subscribe((num) => {
        this.refresh();
      });
    this.refresh();
  }

  private disable_autorefresh() {
    if (this.auto_refresh_subscription) {
      this.auto_refresh_subscription.unsubscribe();
      this.auto_refresh_subscription = null;
    }
  }

  toggle_autorefresh() {
    if (this.auto_refresh_subscription) {
      this.disable_autorefresh();
    } else {
      this.enable_autorefresh();
    }
  }

  format_precondition(cond: GroundedFormula, indent: string = ''): string {
    let s = indent;
    if (cond.kind === 'GroundedFormula') {
      if (! cond['is-satisfied']) {
        s += '! ';
      }
      if(cond.type != 'atom') {
        s += '(';
        switch (cond.type) {
          case 'conjunction':
            s += 'AND';
            break;
          case 'disjunction':
            s += 'OR';
            break;
          case 'negation':
            s += 'NOT';
            break;
        }
        s += ' ';
        for (const c of cond.child) {
          s += '\\l' + this.format_precondition(c, indent + '&nbsp;&nbsp;&nbsp;');
        }
        s += ')';
      } else {
        for (const c of cond.child) {
          s += this.format_precondition(c, indent);
        }
      }
    } else {
      if (! cond['is-satisfied']) {
        s += '! ';
      }
      s += '(';
      s += `${cond.name}`;
      for (const p of cond['param-values']) {
        s += ' ' + p;
      }
      s += ')';
    }
    return s;
  }

  create_goals_graph() {
    let graph = 'digraph {\n' +
      '  graph [fontsize=9];\n' +
      '  node [fontsize=9];\n' +
      '  edge [fontsize=9];\n\n'+
      '  rankdir=LR\n';//+              //Arrange Graph from left to right
     // '  landscape=true\n';          //Putout graph in landscape mode
    if (! this.goals) {
      graph += '  "no goals"';
    } else {
      for (const g of this.goals) {
        const shape = g.type === 'ACHIEVE' ? 'box' : 'ellipse';
        let color = '';

        switch (g.mode) {
          case 'SELECTED':   color = '#eeeeee'; break;
          case 'EXPANDED':   color = '#FFE082'; break;
          case 'COMMITTED':  color = '#FFF59D'; break;
          case 'DISPATCHED': color = '#90CAF9'; break;
          case 'FINISHED':
          case 'EVALUATED':
            switch (g.outcome) {
              case 'COMPLETED': color = '#A5D6A7'; break;
              case 'FAILED':    color = '#EF9A9A'; break;
              case 'REJECTED':  color = '#FFCC80'; break;
            }
            break;
          case 'RETRACTED':
            switch (g.outcome) {
              case 'COMPLETED': color = '#A5D6A7'; break;
              case 'FAILED':    color = '#EF9A9A'; break;
              case 'REJECTED':  color = '#FFCC80'; break;
              default:          color = '#eeeeee'; break;
            }
            break;
          default:
        }

        let node_label = `<table border="0" cellspacing="2"><tr><td colspan="2" align="center"><b>${g.id}</b></td></tr>`;
        // node_label += `<tr><td align="left"><font color="#444444">Mode:</font></td>` +
        //   `<td align="left"><font color="#444444">${this.icon_tooltip(g)}</font></td></tr>`;

        // if (g['class'] && g['class'] !== '') {
        //   node_label += `<tr><td align="left"><font color="#444444">Class:</font></td>` +
        //     `<td align="left"><font color="#444444">${g['class']}</font></td></tr>`;
        //}
        // if (g['sub-type'] && g['sub-type'] !== '') {
        //   node_label += `<tr><td align="left"><font color="#444444">Sub-type:</font></td>` +
        //     `<td align="left"><font color="#444444">${g['sub-type']}</font></td></tr>`;
        // }
        // if (g.priority > 0) {
        //   node_label += `<tr><td align="left"><font color="#444444">Priority:</font></td>` +
        //     `<td align="left"><font color="#444444">${g.priority}</font></td></tr>`;
        // }
        // if (g['sub-type'] && g['sub-type'] === 'RETRY-SUBGOAL') {
        //   if (g.parameters.length === 2 && g.parameters[0] === 'max-tries' &&
        //       g.meta.length === 2 && g.meta[0] === 'num-tries') {
        //     node_label += `<tr><td align="left"><font color="#444444">Tries:</font></td>` +
        //       `<td align="left"><font color="#444444">${g.meta[1]}/${g.parameters[1]}</font></td></tr>`;
        //   }
        // }
        // if (g['parameters'] && g['parameters'].length > 0) {
        //   node_label += `<tr><td align="left"><font color="#444444">Params:</font></td>` +
        //     `<td align="left"><font color="#444444">${g['parameters'].join(' ')}</font></td></tr>`;
        //}
        if (g['error'] && g['error'].length > 0) {
          node_label += `<tr><td align="left"><font color="#444444">Error:</font></td>` +
            `<td align="left"><font color="#ff0000">${g['error'].join(' ')}</font></td></tr>`;
        }
        if (g['required-resources'] && g['required-resources'].length > 0) {
          node_label += `<tr><td align="left"><font color="#444444">Req Resrc:</font></td>` +
            `<td align="left"><font color="#444444">`;
          if (g.mode === 'COMMITTED') {
            const values = [];
            for (const r of g['required-resources']) {
              if (g['acquired-resources'].indexOf(r) === -1) {
                values.push(`<font color="#ff0000">${r}</font>`);
              } else {
                values.push(r);
              }
            }
            node_label += values.join(', ');
          } else {
            node_label += g['required-resources'].join(', ');
          }
          node_label += '</font></td></tr>';
        }
        if (g['acquired-resources'] && g['acquired-resources'].length > 0) {
          const res_color = (g.mode === 'RETRACTED') ? '#ff0000' : '#444444';
          node_label += `<tr><td align="left"><font color="#444444">Acq Resrc:</font></td>` +
            `<td align="left"><font color="${res_color}">${g['acquired-resources'].join(', ')}</font></td></tr>`;
        }
        node_label += '</table>';
//TODO lable raus
//TODO Graphviz schmaler so das auf einen bild passt
//schnellen print ohne make
//ansonsten make deploy in fawkes/src/plugins/webview/frontend
//goalclass reicht
//ordnerstruktur
        graph += `  "${g.id}" [label=<${node_label}>, tooltip="${g.id}", href="/clips-executive/goal/${g.id}", shape=${shape}`;
        if (color !== '') {
          graph += `, style="filled", fillcolor="${color}"`;
        }
        graph += '];\n';
        if (g.parent) {
          graph += `  "${g.parent}" -> "${g.id}";\n`;
        }

        const plans = this.plans.filter(p => p.goal_id === g.id && p.plan);
        for (const p of plans) {
          graph +=
            `  subgraph "cluster_${p.goal_id}__${p.plan_id}" {\n` +
            `    label="${p.plan_id}";\n` +
            `    style=filled; fillcolor="#efefef";\n` +
            `    node [shape=invhouse,style=filled];\n` +
            `    edge [labelangle=290,labeldistance=6.0,labeljust=l];\n`+
            '  rankdir=LR\n';
          let prev = p.goal_id;
          for (const a of p.plan.actions) {
            let bgcolor = '#ffffff';
            let prec_string = '';
            if (a.preconditions) {
              prec_string = this.format_precondition(a.preconditions);
            }
            switch (a.state) {
              case 'WAITING':
              case 'EXECUTION-SUCCEEDED':
              case 'SENSED-EFFECTS-WAIT':
              case 'SENSED-EFFECTS-HOLD':
              case 'EFFECTS-APPLIED':
                bgcolor = '#feffab';
                break;
              case 'EXECUTION-FAILED':
              case 'FAILED':
                bgcolor = '#ff9c9c';
                break;
              case 'FINAL':
                bgcolor = '#ccffcc';
                break;
            }
            /*
            let label = "";
            if (prec_string !== "" && prec_string !== "()" && prec_string !== "TRUE") {
              label = `{ ${prec_string} | ${a["operator-name"]} }`;
            } else {
              label = a["operator-name"];
            }
            */
            const label = a['operator-name'];

            graph += `    "${p.goal_id}__${p.plan_id}__${a.id}" [label="${label}",fillcolor="${bgcolor}"];\n`;
            graph += `    "${prev}" -> "${p.goal_id}__${p.plan_id}__${a.id}" ` +
              `[headlabel="${prec_string}"];\n`;

            prev = `${p.goal_id}__${p.plan_id}__${a.id}`;
          }
          graph += ' }\n';
        }
      }
    }
    graph += '}';
    // console.log(`Graph: ${graph}`);
    this.goals_graph = graph;
  }

  svg_updated(svg: string) {
    this.graph_svg_base64 =
      this.sanitizer.bypassSecurityTrustUrl('data:image/svg+xml;base64,' + btoa(svg));
    this.graph_svg_available = true;
  }
}

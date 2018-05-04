
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Component, OnInit, OnDestroy } from '@angular/core';
import { Router } from '@angular/router';
import { MatTableDataSource } from '@angular/material';

import { BackendConfigurationService } from '../../../services/backend-config/backend-config.service';
import { ClipsExecutiveApiService } from '../services/api.service';
import { Goal } from '../models/Goal';
import { Plan } from '../models/Plan';
import { DomainOperator } from '../models/DomainOperator';
import { DomainPrecondition } from '../models/DomainPrecondition';
import { DomainPreconditionAtom } from '../models/DomainPreconditionAtom';
import { DomainPreconditionCompound } from '../models/DomainPreconditionCompound';

import { Observable, interval, forkJoin } from 'rxjs';

@Component({
  selector: 'clips-executive-goal-list',
  templateUrl: './goal-list.component.html',
  styleUrls: ['./goal-list.component.scss']
})
export class GoalListComponent implements OnInit, OnDestroy {

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

  zero_message = "No goals received.";
  
  constructor(private readonly api_service: ClipsExecutiveApiService,
              private router : Router,
              private backendcfg: BackendConfigurationService)
  {}

  ngOnInit() {
    this.refresh_domain();
    this.refresh();
    this.backend_subscription = this.backendcfg.backend_changed.subscribe((b) => { this.refresh() });
  }

  ngOnDestroy()
  {
    this.backend_subscription.unsubscribe();
    this.backend_subscription = null;
    this.disable_autorefresh();
  }

  refresh()
  {
    this.loading = true;
    this.zero_message = "Retrieving goals";

    this.api_service.list_goals().subscribe(
      (goals) => {
        console.log("Goals", goals)
        let plans = [];
        goals
          .filter(g => g.plans.length > 0)
          .forEach(g => g.plans.forEach(p => plans.push({goal_id: g.id, plan_id: p})));

        this.dataSource.data = this.process_tree(goals);
        if (this.dataSource.data.length == 0) {
          this.zero_message = "Executive has currently no goals";
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
        if (err.status == 0) {
          this.zero_message="API server unavailable. Robot down?";
        } else {
          this.zero_message=`Failed to retrieve goals: ${err.error}`;
        }
        this.loading = false;
      }
    );
  }

  refresh_domain()
  {
    this.api_service.list_domain_operators()
      .subscribe(
        (operators) => { this.operators = operators;},
        (err) => { console.log("Failed to receive domain data"); }
      );
  }

  refresh_plans()
  {
    this.loading = true;
    this.zero_message = "Retrieving plans";

    forkJoin(
      // forkJoin: retrieve all plans associated to goal in parallel
      this.plans.map((plan) => {
        return this.api_service.get_plan(plan.goal_id, plan.plan_id)
      })
    )
    .subscribe(
      (plans : Plan[]) => {
        for (let i = 0; i < plans.length; ++i) {
          console.log("Got plan", this.plans[i].goal_id, this.plans[i].plan_id)
          this.plans[i].plan = plans[i];
        }
        console.log("Plans", this.plans)
        this.create_goals_graph();
        this.loading = false;
      },
      (err) => {
        console.log("Failed to retrieve plans");
        this.create_goals_graph();
        this.loading = false;
      }
    );
  }

  recursive_add_goals(l, level : number,
                      goals : Goal[], sub_goals : Map<string, Goal[]>)
  {
    for (let g of goals) {
      l.push({goal: g, level: level, width: 16*level});
      if (g.id in sub_goals) {
        this.recursive_add_goals(l, level+1, sub_goals[g.id], sub_goals);
      }
    }
  }

  process_tree(data: Goal[])
  {
    let rv = []

    let top_goals : Goal[] = [];
    let sub_goals : Map<string, Goal[]> = new Map();
    for (let g of data) {
      if (! g.parent || g.parent == "") {
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
  
  icon_name(goal : Goal) : string
  {
    switch (goal.mode) {
      case "FORMULATED": return "note_add";
      case "SELECTED":   return "build";
      case "EXPANDED":   return "assignment";
      case "COMMITTED":   return "assignment_turned_in";
      case "DISPATCHED":   return "loop";
      case "FINISHED":
      {
        switch (goal.outcome) {
          case "COMPLETED": return "check";
          case "FAILED": return "error_outline";
          default: return "help";
        }
      }
      case "EVALUATED":
      {
        switch (goal.outcome) {
          case "COMPLETED": return "check_circle";
          case "FAILED": return "error";
          default: return "help";
        }
      }
      case "REJECTED":   return "report";
      default: return "help_outline";
    }
  }

  icon_class(goal : Goal) : string
  {
    switch (goal.mode) {
      case "FORMULATED": return "ff-muted";
      case "SELECTED":   return "ff-primary";
      case "EXPANDED":   return "ff-primary";
      case "COMMITTED":   return "ff-primary";
      case "DISPATCHED":   return "ff-primary";
      case "FINISHED":
      {
        switch (goal.outcome) {
          case "COMPLETED": return "ff-success";
          case "FAILED": return "ff-error";
          default: return "ff-warning";
        }
      }
      case "EVALUATED":
      {
        switch (goal.outcome) {
          case "COMPLETED": return "ff-success";
          case "FAILED": return "ff-error";
          default: return "ff-warning";
        }
      }
      case "REJECTED":   return "ff-warning";
      default: return "ff-warning";
    }
  }

  goto_goal(goal : Goal)
  {
    this.router.navigate(['/clips-executive/goal/', goal.id]);
  }

  show_zero_state() : boolean
  {
    return ! this.dataSource.data || this.dataSource.data.length == 0;
  }

  private enable_autorefresh()
  {
    if (this.auto_refresh_subscription)  return;
    this.auto_refresh_subscription =
      interval(2000).subscribe((num) => {
        this.refresh();
      });
    this.refresh();
  }

  private disable_autorefresh()
  {
    if (this.auto_refresh_subscription) {
      this.auto_refresh_subscription.unsubscribe();
      this.auto_refresh_subscription = null;
    }
  }

  toggle_autorefresh()
  {
    if (this.auto_refresh_subscription) {
      this.disable_autorefresh();
    } else {
      this.enable_autorefresh();
    }
  }

  format_precondition(cond : DomainPrecondition, indent: string = "") : string
  {
    let s = indent;
    if (cond.kind === 'DomainPreconditionCompound') {
      let compound = cond as DomainPreconditionCompound;
      if (! cond['is-satisfied']) {
        s += "! ";
      }
      s += "(";
      switch (compound.type) {
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
      for (let e of compound.elements) {
        s += '\\l' + this.format_precondition(e, indent+"&nbsp;&nbsp;&nbsp;");
      }
      s += ')';
    } else {
      let atom = cond as DomainPreconditionAtom;
      if (! cond['is-satisfied']) {
        s += "! ";
      }
      s += '(';
      s += `${atom.predicate}`;
      for (let p of atom['param-values']) {
        s += ' ' + p;
      }
      s += ')';
    }
    return s;
  }

  create_goals_graph()
  {
    let graph = 'digraph {\n'+
      '  graph [fontsize=10];\n'+
      '  node [fontsize=10];\n'+
      '  edge [fontsize=10];\n\n';
    if (! this.goals) {
      graph += '  "no goals"';
    } else {
      for (let g of this.goals) {
        let shape = g.type == 'ACHIEVE' ? 'ellipse' : 'box';
        let color = '';

        switch (g.mode) {
          case "SELECTED":   color='#eeeeee'; break;
          case "EXPANDED":   color='#FFE082'; break;
          case "COMMITTED":  color='#FFF59D'; break;
          case "DISPATCHED": color='#90CAF9'; break;
          case "FINISHED":
          case "EVALUATED":
            switch (g.outcome) {
              case "COMPLETED": color='#A5D6A7'; break;
              case "FAILED":    color='#EF9A9A'; break;
            }
            break;
          case "REJECTED":   color='#FFCC80'; break;
          default:;
        }
        graph += `  "${g.id}" [href="/clips-executive/goal/${g.id}", shape=${shape}`;
        if (color != '') {
          graph += `, style="filled", fillcolor="${color}"`;
        }
        graph += "];\n";
        if (g.parent) {
          graph += `  "${g.parent}" -> "${g.id}";\n`;
        }

        let plans = this.plans.filter(p => p.goal_id == g.id && p.plan);
        for (let p of plans) {
          graph +=
            `  subgraph "cluster_${p.goal_id}__${p.plan_id}" {\n`+
            `    label="${p.plan_id}";\n`+
            `    style=filled; fillcolor="#efefef";\n`+
            `    node [shape=invhouse,style=filled];\n`+
            `    edge [labelangle=290,labeldistance=6.0,labeljust=l];\n`;
          let prev = p.goal_id;
          for (let a of p.plan.actions) {
            let bgcolor = '#ffffff';
            let prec_string = "";
            if (a.preconditions && a.preconditions.length > 0) {
              prec_string = this.format_precondition(a.preconditions[0]);
            }
            switch (a.status) {
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
            if (prec_string != "" && prec_string != "()" && prec_string != "TRUE") {
              label = `{ ${prec_string} | ${a["operator-name"]} }`;
            } else {
              label = a["operator-name"];
            }
            */
            let label = a["operator-name"];

            graph += `    "${p.goal_id}__${p.plan_id}__${a.id}" [label="${label}",fillcolor="${bgcolor}"];\n`;
            graph += `    "${prev}" -> "${p.goal_id}__${p.plan_id}__${a.id}" `+
              `[headlabel="${prec_string}"];\n`;

            prev = `${p.goal_id}__${p.plan_id}__${a.id}`;
          }
          graph += "  }\n";
        }
      }
    }
    graph += "}";
    console.log(`Graph: ${graph}`);
    this.goals_graph = graph;
  }
}

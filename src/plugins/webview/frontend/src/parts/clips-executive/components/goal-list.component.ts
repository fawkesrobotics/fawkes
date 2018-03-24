
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Component, OnInit } from '@angular/core';
import { Router } from '@angular/router';
import { MatTableDataSource } from '@angular/material';

import { ClipsExecutiveApiService } from '../services/api.service';
import { Goal } from '../models/Goal';

import { Observable } from 'rxjs/Observable';
import 'rxjs/add/observable/interval';

@Component({
  selector: 'clips-executive-goal-list',
  templateUrl: './goal-list.component.html',
  styleUrls: ['./goal-list.component.scss']
})
export class GoalListComponent implements OnInit {

  dataSource = new MatTableDataSource();
  displayedColumns = ['mode', 'id', 'type', 'class'];

  data_received = false;
  auto_refresh_subscription = null;

  loading = false;

  zero_message = "No goals received.";
  
  constructor(private readonly api_service: ClipsExecutiveApiService,
              private router : Router)
  { }

  ngOnInit() {
    this.refresh();
  }

  refresh()
  {
    this.loading = true;
    this.zero_message = "Retrieving goals";

    this.api_service.list_goals().subscribe(
      (goals) => {
        this.dataSource.data = this.process_tree(goals);
        if (this.dataSource.data.length == 0) {
          this.zero_message = "Executive has currently no goals";
        }
        this.loading = false;
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
      Observable.interval(2000).subscribe((num) => {
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
}

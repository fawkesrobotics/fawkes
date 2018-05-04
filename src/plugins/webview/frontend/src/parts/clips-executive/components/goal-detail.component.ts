
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Component, OnInit, OnDestroy } from '@angular/core';
import { Router, ActivatedRoute, ParamMap } from '@angular/router';
import { Observable, interval, forkJoin } from 'rxjs';
import { map, tap, switchMap } from 'rxjs/operators';

import { BackendConfigurationService } from '../../../services/backend-config/backend-config.service';
import { ClipsExecutiveApiService } from '../services/api.service';
import { Goal } from '../models/Goal';
import { Plan } from '../models/Plan';
import { PlanAction } from '../models/PlanAction';
import { DomainOperator } from '../models/DomainOperator';
import { DomainPrecondition } from '../models/DomainPrecondition';
import { DomainPreconditionAtom } from '../models/DomainPreconditionAtom';
import { DomainPreconditionCompound } from '../models/DomainPreconditionCompound';


@Component({
  selector: 'clips-executive-goal-detail',
  templateUrl: './goal-detail.component.html',
  styleUrls: ['./goal-detail.component.scss']
})
export class GoalDetailComponent implements OnInit, OnDestroy {

  private backend_subscription = null;

  constructor(private route: ActivatedRoute,
              private api_service : ClipsExecutiveApiService,
              private backendcfg: BackendConfigurationService)
  {}

  id = "";
  loading_goal   = false;
  loading_plans  = false;

  zero_message_goal  = "Goal not available";
  zero_message_plans = "Plans or domain info not available";
  
  goal: Goal = null;
  plans: Plan[] = [];
  domain = null;

  auto_refresh_subscription = null;
  auto_refresh_tries = 0;

  displayedPlanActionColumns = [ "operator_name", "params", "status", "executable", "preconditions" ];

  ngOnInit() {
    this.refresh_domain();
    this.refresh_goal();
    this.backend_subscription = this.backendcfg.backend_changed.subscribe((b) => {
      this.refresh_domain();
      this.refresh_goal();
    });
  }

  ngOnDestroy()
  {
    this.backend_subscription.unsubscribe();
    this.backend_subscription = null;
    this.disable_autorefresh();
  }

  refresh_domain()
  {
    forkJoin([
      // forkjoin here to allow for requesting multiple items
      this.api_service.list_domain_operators(),
    ])
    .pipe(
      map((data: any[]) => {
        return { operators: data[0] };
      })
    )
    .subscribe(
      (domain) => {
        console.log("Received domain data");
        this.domain = domain;
      },
      (err) => {
        console.log("Failed to receive domain data");
      }
    );
  }
  
  refresh_goal()
  {
    this.loading_goal = true;
    this.zero_message_goal = "Retrieving goal";

    this.route.paramMap
      .pipe(
        tap((params: ParamMap) => this.id = params.get('id')),
        switchMap((params: ParamMap) =>
                  this.api_service.get_goal(params.get('id'))))
      .subscribe(
        (goal) => {
          this.goal = goal;
          if (! this.domain) {
            this.refresh_domain();
          }
          this.loading_goal = false;
          this.refresh_plans();
        },
        (err) => {
          this.goal = null;
          this.loading_goal = false;
          this.domain = null; 
          if (err.status == 0) {
            this.zero_message_goal="API server unavailable. Robot down?";
          } else {
            this.zero_message_goal=`Failed to retrieve goal: ${err.error}`;
          }
        }
      );
  }

  refresh_plans()
  {
    if (this.goal && this.goal.plans && this.goal.plans.length > 0) {
      this.loading_plans = true;
      this.zero_message_plans = "Retrieving plans";
      forkJoin(
        // forkJoin: retrieve all plans associated to goal in parallel
        this.goal.plans.map((plan_id: string) => {
          return this.api_service.get_plan(this.goal.id, plan_id)
          /*
            .map((plan: Plan) => {return plan;})
            .catch((err) => {
            this.domain = null;
            this.refresh_goal();
            return Observable.of([] as Plan[]);
            });
          */
          })
        )
        .subscribe(
          (plans : Plan[]) => {
            let incompatible = false;
            if (! this.plans || this.plans.length != plans.length) {
              incompatible = true;
            } else {
              for (let i = 0; i < this.plans.length; ++i) {
                if (this.plans[i].id != plans[i].id) {
                  incompatible = true;
                  break;
                }
              }
            }
            if (incompatible) {
              this.plans = plans;
            } else {
              // only update actions
              let need_refresh_goal = false;
              for (let i = 0; i < this.plans.length; ++i) {
                this.plans[i].actions = plans[i].actions;
                this.plans[i].actions.length = plans[i].actions.length;
                if (this.plans[i].actions[this.plans[i].actions.length - 1].status == 'FINAL' &&
                    this.goal.mode != 'FINISHED' && this.goal.mode != 'EVALUATED')
                {
                  need_refresh_goal = true;
                }
              }
              if (this.plans.length == 0) {
                this.zero_message_plans = "Executive has currently no plans for this goal";
              }
              if (need_refresh_goal) {
                this.refresh_goal();
              }
            }
            this.loading_plans = false;
          },
          (err) => {
            this.plans = [];
            this.domain = null;
            this.loading_plans = false;
            if (err.status == 0) {
              this.zero_message_plans="API server unavailable. Robot down?";
            } else {
              this.zero_message_plans=`Failed to retrieve plans: ${err.error}`;
            }
            this.refresh_goal();
          }
        );
    } else {
      this.plans = [];
      if (! this.goal) {
        this.zero_message_plans = "Goal has not been loaded, yet.";
      } else if (! this.goal.plans || this.goal.plans.length == 0) {
        this.zero_message_plans = "Goal does not list any plans.";
      }
    }
  }

  domain_operator(domain: any, name: string)
  {
    return domain.operators.find(op => op.name === name);
  }
  
  action_status_classes(action: PlanAction)
  {
    return {
      "ff-bg-warning":    ['WAITING','EXECUTION-SUCCEEDED','SENSED-EFFECTS-WAIT',
                           'SENSED-EFFECTS-HOLD','EFFECTS-APPLIED'].includes(action.status),
      "ff-bg-background": action.status == 'RUNNING',
      "ff-bg-error":    ['EXECUTION-FAILED', 'FAILED'].includes(action.status),
      "ff-bg-success":    action.status == 'FINAL'
    };
  }

  recursive_add_preconditions(l, conds : DomainPrecondition[], level : number = 0)
  {
    for (let c of conds) {
      l.push({cond: c, level: level, width: 16*level});
      if (c.kind === 'DomainPreconditionCompound') {
        let compound = c as DomainPreconditionCompound;
        this.recursive_add_preconditions(l, compound.elements, level+1);
      }
    }
  }
  
  format_preconditions(action: PlanAction)
  {
    let rv = []
    this.recursive_add_preconditions(rv, action.preconditions);
    return rv;
  }

  private enable_autorefresh()
  {
    if (this.auto_refresh_subscription)  return;
    this.auto_refresh_subscription =
      interval(2000).subscribe((num) => {
        if (!this.loading_goal && (++this.auto_refresh_tries < 10)) {
          this.auto_refresh_tries = 0;
          this.refresh_goal();
        }
      });
    this.refresh_goal();
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

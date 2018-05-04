
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Component, OnInit, OnDestroy, ViewChild } from '@angular/core';
import { Observable, interval } from 'rxjs';
import { SessionStorage } from 'ngx-store';

import { BackendConfigurationService } from '../../../services/backend-config/backend-config.service';
import { LockoutService } from '../../../services/lockout/lockout.service';
import { BehaviorEngineApiService } from '../services/api.service';
import { SkillCall } from '../models/SkillCall';

@Component({
  selector: 'skiller-overview',
  templateUrl: './overview.component.html',
  styleUrls: ['./overview.component.scss']
})
export class SkillerOverviewComponent implements OnInit, OnDestroy {

  @ViewChild('f') form;

  loading = false;
  auto_refresh_subscription = null;
  @SessionStorage() selected_skill = "active";
  skill = null;
  skills = null;
  zero_message = "No graph has been retrieved";
  message_id = 0;

  private backend_subscription = null;

  constructor(private api_service: BehaviorEngineApiService,
              public lockout: LockoutService,
              private backendcfg: BackendConfigurationService)
  {}

  ngOnInit() {
    this.refresh();
    this.refresh_skills();
    this.backend_subscription = this.backendcfg.backend_changed.subscribe((b) => {
      this.refresh();
      this.refresh_skills();
      this.select_skill('active');
    });
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
    this.zero_message = "Retrieving graph";

    this.api_service.get_skill(this.selected_skill).subscribe(
      (skill) => {
        this.skill = skill;
        if (this.skill.graph == "") {
          this.zero_message = "No graph in blackboard";
        }
        if (this.selected_skill == 'active' &&
            this.message_id > 0 && skill.msg_id == this.message_id &&
            (skill.status == 'FINAL' || skill.status == 'FAILED'))
        {
          this.message_id = 0;
          this.disable_autorefresh();
        }
        this.loading = false;
      },
      (err) => {
        this.skill = null;
        if (err.status == 0) {
          this.zero_message="API server unavailable. Robot down?";
        } else {
          this.zero_message=`Failed to retrieve graph: ${err.error}`;
        }
        this.loading = false;
      }
    );
  }

  refresh_skills()
  {
    this.loading = true;

    this.api_service.list_skills().subscribe(
      (skills) => {
        this.skills = skills;
        this.loading = false;
      },
      (err) => {
        this.skills = null;
        this.loading = false;
      }
    );
  }

  select_skill(skill_name: string)
  {
    this.selected_skill = skill_name;
    this.refresh();
  }

  private enable_autorefresh()
  {
    if (this.auto_refresh_subscription)  return;
    this.auto_refresh_subscription =
      interval(1000).subscribe((num) => {
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

  exec_skill(skill_string: string)
  {
    let call: SkillCall = { kind: 'SkillCall', apiVersion: SkillCall.API_VERSION,
                            skill_string: skill_string };
    this.message_id = 0;
    this.api_service.exec_skill(call).subscribe(
      (skill) => {
        this.message_id = skill.msg_id;
      },
      (err) => {
        console.log(`Skill execution of ${skill_string} failed: ${JSON.stringify(err)}`);
        this.disable_autorefresh();
      }
    );
    this.enable_autorefresh();
  }

  tooltip_skillstring()
  {
    if (this.lockout.enabled) {
      return "Lockout enabled";
    } else {
      return null;
    }
  }

  tooltip_exec()
  {
    if (this.lockout.enabled) {
      return "Lockout enabled";
    } else if (this.selected_skill == 'active' && this.skill && this.skill.exclusive_controller != 0) {
      return "Exclusive controller registered";
    } else if (!this.form.valid) {
      return "No skill string";
    } else {
      return null;
    }
  }

  stop_skill()
  {
    this.api_service.stop_skill('active').subscribe(
      (response) => {},
      (err) => {
        console.log(`Failed to stop skill execution: ${JSON.stringify(err)}`);
      }
    );
  }

  tooltip_stop()
  {
    if (this.lockout.enabled) {
      return "Lockout enabled";
    } else if (!this.skill) {
      return "No skill running";
    } else {
      if (this.skill.status != 'RUNNING') {
        return "No skill running";
      } else if (this.selected_skill != "active") {
        return "Skill other than active selected";
      } else if (this.skill.exclusive_controller != 0) {
        return "Other exclusive controller registered";
      } else {
        return "Stop executing skill";
      }
    }
  }

  disabled_stop()
  {
    return this.lockout.enabled || !this.skill || this.selected_skill != 'active' ||
      this.skill.status != 'RUNNING' || this.skill.exclusive_controller != 0;
  }

}

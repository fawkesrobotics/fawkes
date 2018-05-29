
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Injectable, EventEmitter } from '@angular/core';
import { LocalStorage } from 'ngx-store';

@Injectable()
export class LockoutService {

  public changed: EventEmitter<boolean>;

  @LocalStorage()
  private lockout_enabled: boolean = false;
  
  constructor()
  {
    this.changed = new EventEmitter();
  }

  toggle()
  {
    this.enabled = ! this.enabled;
  }
  
  set enabled(en: boolean)
  {
    this.lockout_enabled = en;
    this.changed.emit(this.lockout_enabled);
  }

  get enabled(): boolean
  {
    return this.lockout_enabled;
  }
}

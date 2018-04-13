
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// Copyright  2017  The Kubernetes Authors
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import {AfterContentInit, Component, OnInit, ViewChild} from '@angular/core';
import {MatDrawer} from '@angular/material';

import {NavService} from '../../services/nav/service';
import { LockoutService } from '../../services/lockout/lockout.service';

@Component({
  selector: 'ff-nav',
  templateUrl: './template.html',
  styleUrls: ['./style.scss'],
})
export class NavComponent implements AfterContentInit, OnInit {
  @ViewChild(MatDrawer) private readonly nav_: MatDrawer;

  constructor(private readonly navService_: NavService,
              public lockout: LockoutService) {}

  ngOnInit(): void {
    this.navService_.setNav(this.nav_);
  }

  ngAfterContentInit(): void {
    this.nav_.open();
  }
}

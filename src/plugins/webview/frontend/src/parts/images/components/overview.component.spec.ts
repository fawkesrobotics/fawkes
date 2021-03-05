
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { waitForAsync, ComponentFixture, TestBed } from '@angular/core/testing';

import { BlackboardOverviewComponent } from './overview.component';

describe('BlackboardOverviewComponent', () => {
  let component: BlackboardOverviewComponent;
  let fixture: ComponentFixture<BlackboardOverviewComponent>;

  beforeEach(waitForAsync(() => {
    TestBed.configureTestingModule({
      declarations: [ BlackboardOverviewComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(BlackboardOverviewComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});


// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// Copyright  2017  The Kubernetes Authors
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Component, ElementRef, EventEmitter, OnInit, ViewChild } from '@angular/core';
import { Subject } from 'rxjs';
import { debounceTime, distinctUntilChanged } from 'rxjs/operators';

@Component({
  selector: 'ff-card-list-filter',
  templateUrl: './template.html',
  styleUrls: ['style.scss'],
})
export class CardListFilterComponent implements OnInit {
  @ViewChild('filterInput') private readonly filterInput_: ElementRef;
  private hidden_ = true;
  keyUpEvent = new Subject<string>();
  query = '';
  filterEvent: EventEmitter<string> = new EventEmitter<string>();

  ngOnInit(): void {
    this.keyUpEvent
      .pipe(
        debounceTime(500),
        distinctUntilChanged()
      )
      .subscribe(
        this.onFilterTriggered_.bind(this)
      );
  }

  private onFilterTriggered_(newVal: string): void {
    this.query = newVal;
    this.filterEvent.emit(this.query);
  }

  isSearchVisible(): boolean {
    return !this.hidden_;
  }

  switchSearchVisibility(event: Event): void {
    event.stopPropagation();
    this.hidden_ = !this.hidden_;

    if (!this.hidden_) {
      this.focusInput();
    }
  }

  focusInput(): void {
    // Small timeout is required as input is not yet rendered when method is fired right after
    // clicking on filter button.
    setTimeout(() => {
      this.filterInput_.nativeElement.focus();
    }, 150);
  }

  clearInput(event?: Event): void {
    if (event) {
      this.switchSearchVisibility(event);
    } else {
      this.hidden_ = true;
    }

    // Do not call backend if it is not needed
    if (this.query.length > 0) {
      this.query = '';
      this.filterEvent.emit(this.query);
    }
  }
}
